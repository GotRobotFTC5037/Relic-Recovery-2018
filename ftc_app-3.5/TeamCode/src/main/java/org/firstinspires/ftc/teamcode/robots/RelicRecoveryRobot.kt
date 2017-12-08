package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.libraries.MRIColorBeacon
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.min

/**
 * Created by FTC Team 5037 gotrobot?
 */
class RelicRecoveryRobot : MecanumRobot() {

    companion object {
        val CRYPTO_BOX_SPACING = 47.0

        val LEADING_FRONT_CRYPTO_BOX_DISTANCE = 46.0
        val CENTER_FRONT_CRYPTO_BOX_DISTANCE = 64.0
        val TRAILING_FRONT_CRYPTO_BOX_DISTANCE = 82.0

        val LEADING_SIDE_CRYPTO_BOX_DISTANCE = 102.0
        val CENTER_SIDE_CRYPTO_BOX_DISTANCE = 116.0
        val TRAILING_SIDE_CRYPTO_BOX_DISTANCE = 137.0

        private val BALANCING_STONE_ANGLE_THRESHOLD = 4.0
        private val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 2.5

        private val FRONT_DISTANCE_SENSOR_FILTER_ALPHA = 0.50
        private val LEFT_DISTANCE_SENSOR_FILTER_ALPHA = 0.50
        private val RIGHT_DISTANCE_SENSOR_FILTER_ALPHA = 0.50
        private val BACK_DISTANCE_SENSOR_FILTER_ALPHA = 0.15

        private val GLYPH_GRABBER_OPEN_POSITION = 1.0
        private val GLYPH_GRABBER_SMALL_OPEN_POSITION = 0.70
        private val GLYPH_GRABBER_RELEASE_POSITION = 0.60
        private val GLYPH_GRABBER_CLOSED_POSITION = 0.40

        private val GLYPH_DEPLOYER_EXTENDED_POSITION = 0.25
        private val GLYPH_DEPLOYER_RETRACTED_POSITION = 0.90
        private val GLYPH_DEPLOYER_UP = 0.01

        private val OBJECT_DISTANCE_THRESHOLD = 2.0

        private val MAXIMUM_ENCODER_LIFT_POSITION = 3200
        val AUTO_LIFT_FIRST_LEVEL = 1000

        fun getRobotSetupPosition(hardwareMap: HardwareMap): SetupPosition {
            val colorSensor = hardwareMap.colorSensor.get("floor color sensor")
            val backRangeSensor = hardwareMap.analogInput.get("back range sensor")

            val red = colorSensor.red()
            val blue = colorSensor.blue()
            val backObjectDistance = (backRangeSensor.voltage / (5.0 / 512.0)) * 2.54

            return when {
                red < blue && backObjectDistance > 60 -> SetupPosition.FRONT_BLUE
                red < blue && backObjectDistance < 60 -> SetupPosition.BACK_BLUE
                red > blue && backObjectDistance < 60 -> SetupPosition.FRONT_RED
                red > blue && backObjectDistance > 60 -> SetupPosition.BACK_RED
                else -> SetupPosition.UNKNOWN
            }
        }
    }

    private lateinit var liftMotor: DcMotor
    private lateinit var jewelStick: Servo
    private lateinit var leftGlyphGrabber: Servo
    private lateinit var rightGlyphGrabber: Servo
    private lateinit var glyphDeployer: Servo
    private lateinit var floorColorSensor: ColorSensor
    private lateinit var leftRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var rightRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var frontLeftRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var frontRightRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var backRangeSensor: AnalogInput
    private lateinit var liftLimitSwitch: DigitalChannel
    private lateinit var colorBeacon: MRIColorBeacon

    private var colorBeaconThread = thread {}
    private var startingPitch = 0.0
    private var frontLeftObjectDistance = 0.0
    private var frontRightObjectDistance = 0.0
    private var leftObjectDistance = 0.0
    private var rightObjectDistance = 0.0
    private var backObjectDistance = 0.0

    /**
     * Sets up the hardware needed in order to use the robot.
     * @param hardwareMap A HardwareMap object. Usually provided by linearOpMode.hardwareMap.
     */
    override fun setup(hardwareMap: HardwareMap) {
        frontLeftObjectDistance = 0.0
        leftObjectDistance = 0.0
        rightObjectDistance = 0.0

        liftMotor = hardwareMap.dcMotor.get("winch motor")
        liftMotor.direction = DcMotorSimple.Direction.FORWARD
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")
        glyphDeployer = hardwareMap.servo.get("glyph deployer")

        floorColorSensor = hardwareMap.colorSensor.get("floor color sensor")

        frontRightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "front right range sensor")
        frontRightRangeSensor.i2cAddress = I2cAddr.create8bit(0x34)

        frontLeftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "front left range sensor")
        frontLeftRangeSensor.i2cAddress = I2cAddr.create8bit(0x32)

        leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "left range sensor")
        leftRangeSensor.i2cAddress = I2cAddr.create8bit(0x28)

        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "right range sensor")
        rightRangeSensor.i2cAddress = I2cAddr.create8bit(0x46)

        backRangeSensor = hardwareMap.analogInput.get("back range sensor")

        liftLimitSwitch = hardwareMap.digitalChannel.get("limit switch")
        liftLimitSwitch.mode = DigitalChannel.Mode.INPUT

        colorBeacon = MRIColorBeacon()
        colorBeacon.init(hardwareMap, "color beacon")
        colorBeacon.yellow()

        super.setup(hardwareMap)
    }

    /**
     * Starts processes that are critical for the robot to operate properly in autonomous.
     */
    fun start() {
        startUpdatingRangeSensors()
        startUpdatingDriveMotorPowers()
        startingPitch = this.pitch
    }

    /**
     * Drives the robot forward or backwards for a certain amount of time.
     * @param milliseconds The length of time in seconds to drive.
     * @param power The power to run the motors at when driving.
     */
    fun timeDrive(milliseconds: Long, power: Double = 0.50) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            linearOpMode.sleep(milliseconds)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot forwards or backwards until the wall is a certain distance away from the front.
     * @param distance The distance the robot should be from a front object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromForwardObject(distance: Double, power: Double = 0.15) {

        // Check to see if the opmode is still active.
        if (!linearOpMode.isStopRequested) {
            // Get the current distance for future reference.
            val currentDistance = frontLeftObjectDistance

            when {
                currentDistance > distance -> {
                    setDrivePower(Math.abs(power))
                    while (frontLeftObjectDistance > distance && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(10)
                    }
                }

                currentDistance < distance -> {
                }
            }
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot left or right until the wall is a certain distance away from the left.
     * @param distance The distance the robot should be from a left object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromLeftObject(distance: Double, power: Double = 0.45) {
        val currentDistance = leftObjectDistance

        when {
            currentDistance > distance -> {
                setStrafePower(-Math.abs(power))
                while (leftObjectDistance > distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                setStrafePower(Math.abs(power))
                while (leftObjectDistance < distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val leftDistance = leftObjectDistance
        if ((distance + OBJECT_DISTANCE_THRESHOLD < leftDistance || distance - OBJECT_DISTANCE_THRESHOLD > leftDistance)
                && linearOpMode.opModeIsActive()) {
            driveToDistanceFromLeftObject(distance, 0.45)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot left or right until the wall is a certain distance away from the right.
     * @param distance The distance the robot should be from a right object.
     * @param power The power to run the motors at when driving.
     */
    fun driveToDistanceFromRightObject(distance: Double, power: Double = 0.45) {
        val currentDistance = rightObjectDistance

        when {
            currentDistance > distance -> {
                setStrafePower(Math.abs(power))
                while (rightObjectDistance > distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                setStrafePower(-Math.abs(power))
                while (rightObjectDistance < distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        linearOpMode.telemetry.update()

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val rightDistance = rightObjectDistance
        if ((distance + OBJECT_DISTANCE_THRESHOLD < rightDistance || distance - OBJECT_DISTANCE_THRESHOLD > rightDistance)
                && !linearOpMode.isStopRequested) {
            driveToDistanceFromRightObject(distance, 0.45)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot off of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     * TODO: Put a safety in this function that stops the robot if it goes for a certain amount of time without changes in pitch.
     */
    fun driveOffBalancingStone(power: Double = 0.15) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            when {
                power > 0 -> {
                    while (startingPitch - pitch < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    while (startingPitch - pitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                power < 0 -> {
                    while (startingPitch - pitch > -BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }

                    while (startingPitch - pitch <= -BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                else -> return
            }
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot on of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     * TODO: Put a safety in this function that stops the robot if it goes for a certain amount of time without changes in pitch.
     */
    fun driveOnBalancingStone(power: Double = 0.50) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            while (abs(startingPitch - pitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                linearOpMode.sleep(10)
            }
        }

        stopAllDriveMotors()
    }

    /**
     * Sets the power of the lift's witch motor. Ignores setting power if the robot's lift is at its limits.
     * @param power The power to run the lift at.
     */
    fun setLiftWinchPower(power: Double) {
        if ((power >= 0 && liftMotor.currentPosition <= MAXIMUM_ENCODER_LIFT_POSITION) || (!liftIsLowered() && power < 0))
            liftMotor.power = power
        else
            liftMotor.power = 0.0
    }

    /**
     * Sets the lift at a specific location.
     * @param position The position in encoder units that the lift should go to.
     * @param power The power that the lift motor should lift witch motor at.
     */
    fun setLiftPosition(position: Int, power: Double = 0.5) {
        if (!linearOpMode.isStopRequested) {
            when {
                liftMotor.currentPosition < position -> {
                    setLiftWinchPower(Math.abs(power))

                    while (liftIsLowered() && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(20)
                    }

                    val targetPosition = liftMotor.currentPosition + position
                    while (liftMotor.currentPosition < targetPosition && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(50)
                    }
                }

                liftMotor.currentPosition > position -> {
                    setLiftWinchPower(-Math.abs(power))

                    while (liftIsLowered() && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(20)
                    }

                    val targetPosition = liftMotor.currentPosition + position
                    while (liftMotor.currentPosition > targetPosition && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(50)
                    }
                }
            }
        }

        setLiftWinchPower(0.0)
    }

    /**
     * @param power The power to run the lift witch motor at.
     * Lowers the lift until it reaches the bottom.
     */
    fun dropLift(power: Double = 0.30) {
        if (!linearOpMode.isStopRequested) {
            setLiftWinchPower(-Math.abs(power))
            while (linearOpMode.opModeIsActive() && !liftIsLowered()) {
                linearOpMode.sleep(10)
            }

            setLiftWinchPower(0.0)
            liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    /**
     * Determines if the lift is at the bottom.
     * @return True, if the lift is at the bottom.
     */
    private fun liftIsLowered() = liftLimitSwitch.state || liftMotor.currentPosition <= -5

    /**
     * Sets the position of the glyph grabbers.
     * @param position The position to set the grabbers at.
     */
    private fun setGlyphGrabbersPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = 1 - position
    }

    /**
     * Sets the glyph grabbers to the open position.
     * @param delay The milliseconds to wait after opening the glyph grabbers.
     */
    fun openGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_OPEN_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph grabbers to the open position.
     * @param delay The milliseconds to wait after opening the glyph grabbers.
     */
    fun smallOpenGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_SMALL_OPEN_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph grabbers to the release position.
     * @param delay The milliseconds to wait after releasing the glyph grabbers.
     */
    fun releaseGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_RELEASE_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph grabbers to the closed position.
     * @param delay The milliseconds to wait after closing the glyph grabbers.
     */
    fun closeGlyphGrabbers(delay: Long = 0) {
        setGlyphGrabbersPosition(GLYPH_GRABBER_CLOSED_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph deployer to the extended position.
     * @param delay The milliseconds to wait after extending the glyph deployer.
     */
    fun extendGlyphDeployer(delay: Long = 0) {
        glyphDeployer.position = GLYPH_DEPLOYER_EXTENDED_POSITION
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph deployer to the retracted position.
     * @param delay The milliseconds to wait after retracting the glyph deployer.
     */
    fun retractGlyphDeployer(delay: Long = 0) {
        glyphDeployer.position = GLYPH_DEPLOYER_RETRACTED_POSITION
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the glyph deployer to the lifted position.
     * @param delay The milliseconds to wait after lifting the glyph deployer.
     */
    fun liftGlyphDeployer(delay: Long = 0) {
        glyphDeployer.position = GLYPH_DEPLOYER_UP
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the jewel stick position.
     * @param position The position to set the jewel stick in.
     */
    private fun setJewelStickPosition(position: Double) {
        jewelStick.position = position
    }

    /**
     * Sets the position of the jewel stick to the raised position.
     * @param delay The milliseconds to wait after raising the jewel stick.
     */
    fun raiseJewelStick(delay: Long = 0) {
        setJewelStickPosition(0.0)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the position of the jewel stick to the lowered position.
     * @param delay The milliseconds to wait after lowering the jewel stick.
     */
    fun lowerJewelStick(delay: Long = 2000) {
        setJewelStickPosition(0.9225)
        linearOpMode.sleep(delay)
    }

    /**
     * Begins updating the values of the range sensors.
     */
    private fun startUpdatingRangeSensors() {
        thread(start = true) {
            while (!linearOpMode.isStopRequested) {
                updateFrontLeftObjectDistance()
                updateFrontRightObjectDistance()
                updateLeftObjectDistance()
                updateRightObjectDistance()
                updateBackObjectDistance()
                Thread.sleep(10)
            }
        }
    }

    /**
     * Updates the distance detected by the front left range sensor.
     */
    private fun updateFrontLeftObjectDistance() {
        val rawDistance = frontLeftRangeSensor.cmUltrasonic()
        frontLeftObjectDistance += FRONT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - frontLeftObjectDistance)
    }

    /**
     * Updates the distance detected by the front right range sensor.
     */
    private fun updateFrontRightObjectDistance() {
        val rawDistance = frontRightRangeSensor.cmUltrasonic()
        frontRightObjectDistance += FRONT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - frontRightObjectDistance)
    }

    /**
     * Updates the distance detected by the left range sensor.
     */
    private fun updateLeftObjectDistance() {
        val rawDistance = leftRangeSensor.cmUltrasonic()
        if (abs(rawDistance - leftObjectDistance) <= 200) {
            leftObjectDistance += LEFT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - leftObjectDistance)
        }
    }

    /**
     * Updates the distance detected by the right range sensor.
     */
    private fun updateRightObjectDistance() {
        val rawDistance = rightRangeSensor.cmUltrasonic()
        if (abs(rawDistance - rightObjectDistance) <= 200) {
            rightObjectDistance += RIGHT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - rightObjectDistance)
        }
    }

    /**
     * Updates the distance detected by the back range sensor.
     */
    private fun updateBackObjectDistance() {
        val rawDistance = (backRangeSensor.voltage / (5.0 / 512.0)) * 2.54
        backObjectDistance += BACK_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - backObjectDistance)
    }

    /**
     * Detects a nearby crypto box and delivers a glyph into it assuming that the height of the lift
     * is the height that the glyph should be delivered at.
     */
    fun deliverGlyph() {
        val leftObjectDistance = leftObjectDistance
        val rightObjectDistance = rightObjectDistance
        val heading = heading

        when (heading) {
            in -45..45 -> {
                turn(0.25, 0.0)
                when {
                    leftObjectDistance < rightObjectDistance -> {
                        val leftCryptoBoxDistance = abs(leftObjectDistance - LEADING_FRONT_CRYPTO_BOX_DISTANCE)
                        val centerCryptoBoxDistance = abs(leftObjectDistance - CENTER_FRONT_CRYPTO_BOX_DISTANCE)
                        val rightCryptoBoxDistance = abs(leftObjectDistance - TRAILING_FRONT_CRYPTO_BOX_DISTANCE)
                        val closestCryptoBoxDistance = min(leftCryptoBoxDistance, min(centerCryptoBoxDistance, rightCryptoBoxDistance))

                        val targetLeftDistance: Double = when {
                            leftCryptoBoxDistance == closestCryptoBoxDistance -> LEADING_FRONT_CRYPTO_BOX_DISTANCE
                            centerCryptoBoxDistance == closestCryptoBoxDistance -> CENTER_FRONT_CRYPTO_BOX_DISTANCE
                            rightCryptoBoxDistance == closestCryptoBoxDistance -> TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                            else -> {
                                return
                            }
                        }

                        driveToDistanceFromLeftObject(targetLeftDistance)
                        timeDrive(1000)
                        extendGlyphDeployer()
                        openGlyphGrabbers(250)
                        timeDrive(850, -0.15)
                        retractGlyphDeployer()
                        openGlyphGrabbers()
                        dropLift()
                    }

                    rightObjectDistance < leftObjectDistance -> {

                    }
                }
            }

            in -135..-45 -> {
                turn(0.25, -90.0)
                when {
                    leftObjectDistance < rightObjectDistance -> {

                    }

                    rightObjectDistance < leftObjectDistance -> {

                    }
                }
            }

            in 45..135 -> {
                turn(0.25, 90.0)
                when {
                    leftObjectDistance < rightObjectDistance -> {

                    }

                    rightObjectDistance < leftObjectDistance -> {

                    }
                }
            }

        }

    }

    /**
     * An indicated state or decision of the robot manifested by the color beacon.
     */
    enum class ColorBeaconState {
        IDLE,
        ERROR,
        CALIBRATING,
        DETECTING,
        READY,
        RUNNING
    }

    /**
     * Indicates the state and decisions of the robot with the color beacon.
     */
    fun setColorBeaconState(state: ColorBeaconState) {
        colorBeaconThread.interrupt()

        when (state) {
            ColorBeaconState.IDLE ->
                colorBeaconThread = thread(true, priority = 4) {
                    colorBeacon.off()
                }

            ColorBeaconState.ERROR ->
                colorBeaconThread = thread(true, priority = 4) {
                    colorBeacon.red()
                }

            ColorBeaconState.CALIBRATING ->
                colorBeaconThread = thread(true, priority = 4) {
                    colorBeacon.yellow()
                    linearOpMode.sleep(1000)
                    colorBeacon.off()
                    linearOpMode.sleep(1000)
                }

            ColorBeaconState.DETECTING ->
                colorBeaconThread = thread(true, priority = 4) {
                    colorBeacon.green()
                    linearOpMode.sleep(1000)
                    colorBeacon.off()
                    linearOpMode.sleep(1000)
                }

            ColorBeaconState.READY ->
                colorBeaconThread = thread(true, priority = 4) {
                    colorBeacon.green()
                }

            ColorBeaconState.RUNNING ->
                colorBeaconThread = thread(true, priority = 4) {
                    colorBeacon.blue()
                    linearOpMode.sleep(1000)
                    colorBeacon.off()
                    linearOpMode.sleep(1000)
                }
        }

    }

    /**
     * Prints instructions for using the tele-op OpMode.
     */
    fun printTeleOpInstructions() {
        val telemetry = linearOpMode.telemetry
        telemetry.addLine("Gamepad 1:")
        telemetry.addLine("Drive: Left Joystick")
        telemetry.addLine("Turn: Right Joystick")
        telemetry.addLine("Adjust Perspective: Start + Back")
        telemetry.addLine()
        telemetry.addLine("Gamepad 2:")
        telemetry.addLine("Move Lift: Left Joystick")
        telemetry.addLine("Grab Glyph: A")
        telemetry.addLine("Release Glyph: B")
        telemetry.update()
    }

}