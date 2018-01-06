package org.firstinspires.ftc.teamcode.robots

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.libraries.sensors.RangeSensor
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.opmodes.competition.RelicRecoveryTeleOp
import kotlin.math.abs

/**
 * A class for our robot in the 2017-2018 FTC game, Relic Recovery.
 *
 * @author FTC Team 5037 gotrobot?
 */
class RelicRecoveryRobot : MecanumRobot() {

    companion object {
        val CRYPTO_BOX_SPACING = 47.0

        val LEADING_FRONT_CRYPTO_BOX_DISTANCE = 46.0
        val CENTER_FRONT_CRYPTO_BOX_DISTANCE = LEADING_FRONT_CRYPTO_BOX_DISTANCE + 19.3
        val TRAILING_FRONT_CRYPTO_BOX_DISTANCE = CENTER_FRONT_CRYPTO_BOX_DISTANCE + 19.3

        val LEADING_SIDE_CRYPTO_BOX_DISTANCE = 100.0
        val CENTER_SIDE_CRYPTO_BOX_DISTANCE = LEADING_SIDE_CRYPTO_BOX_DISTANCE + 19.3
        val TRAILING_SIDE_CRYPTO_BOX_DISTANCE = CENTER_SIDE_CRYPTO_BOX_DISTANCE + 19.3

        private val BALANCING_STONE_ANGLE_THRESHOLD = 6.0
        private val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 1.0

        private val GLYPH_GRABBER_OPEN_POSITION = 0.50
        private val GLYPH_GRABBER_SMALL_OPEN_POSITION = 0.66
        private val GLYPH_GRABBER_RELEASE_POSITION = 0.75
        private val GLYPH_GRABBER_CLOSED_POSITION = 1.0

        private val GLYPH_DEPLOYER_EXTENDED_POSITION = 0.25
        private val GLYPH_DEPLOYER_RETRACTED_POSITION = 0.90
        private val GLYPH_DEPLOYER_UP = 0.01

        private val JEWEL_STICK_UP_POSITION = 0.0
        private val JEWEL_STICK_DOWN_POSITION = 0.875

        private val DEFAULT_OBJECT_DISTANCE_TOLERANCE = 2.0

        private val MAXIMUM_ENCODER_LIFT_POSITION = 3350
        val AUTO_LIFT_FIRST_LEVEL = 1000

        /**
         * Determines the current setup position of the robot.
         * @param hardwareMap The instance of HardwareMap from the OpMode that the robot is running on.
         * @return The setup position of the robot.
         */
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

    /**
     * A position that the robot can be setup on on the field.
     */
    enum class SetupPosition {

        FRONT_RED,
        FRONT_BLUE,
        BACK_BLUE,
        BACK_RED,
        UNKNOWN
    }

    /**
     * A position to set the lift to.
     */
    enum class LiftPosition {

        OVEREXTENDED,
        FOURTH_LEVEL,
        THIRD_LEVEL,
        SECOND_LEVEL,
        FIRST_LEVEL,
        BOTTOM_LEVEL
    }

    lateinit var liftMotor: DcMotor
    private lateinit var jewelStick: Servo
    private lateinit var leftGlyphGrabber: Servo
    private lateinit var rightGlyphGrabber: Servo
    private lateinit var glyphDeployer: Servo
    lateinit var leftRangeSensor: RangeSensor
    lateinit var rightRangeSensor: RangeSensor
    lateinit var frontLeftRangeSensor: RangeSensor
    lateinit var frontRightRangeSensor: RangeSensor
    private lateinit var backRangeSensor: RangeSensor
    private lateinit var floorColorSensor: ColorSensor
    private lateinit var liftLimitSwitch: DigitalChannel
    lateinit var jewelConfigurationDetector: JewelConfigurationDetector
    private var startingPitch = 0.0

    /**
     * Sets up the hardware needed in order to use the robot.
     * @param hardwareMap A HardwareMap object. Usually provided by linearOpMode.hardwareMap.
     */
    override fun setup(hardwareMap: HardwareMap) {
        linearOpMode.telemetry.log().add("Setting up the robot.")

        liftMotor = hardwareMap.dcMotor.get("winch motor")
        liftMotor.direction = DcMotorSimple.Direction.REVERSE
        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")
        glyphDeployer = hardwareMap.servo.get("glyph deployer")

        floorColorSensor = hardwareMap.colorSensor.get("floor color sensor")

        frontRightRangeSensor = RangeSensor(linearOpMode, "front right range sensor")
        frontLeftRangeSensor = RangeSensor(linearOpMode, "front left range sensor")
        leftRangeSensor = RangeSensor(linearOpMode, "left range sensor")
        rightRangeSensor = RangeSensor(linearOpMode, "right range sensor")
        backRangeSensor = RangeSensor(linearOpMode, "back range sensor")

        liftLimitSwitch = hardwareMap.digitalChannel.get("limit switch")
        liftLimitSwitch.mode = DigitalChannel.Mode.INPUT

        jewelConfigurationDetector = JewelConfigurationDetector()

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
     * Runs everything that is needed in order to run autonomous.
     */
    fun prepareForAutonomous(linearOpMode: LinearOpMode) {
        linearOpMode.telemetry.log().add("Preparing the robot for autonomous.")

        this.linearOpMode = linearOpMode
        setup(linearOpMode.hardwareMap)

        jewelConfigurationDetector.init(linearOpMode.hardwareMap.appContext, CameraViewDisplay.getInstance())
        jewelConfigurationDetector.enable()

        RelicRecoveryRobotOpModeManager.queueOpMode(linearOpMode, RelicRecoveryTeleOp.OPMODE_NAME)
        RelicRecoveryRobotOpModeManager.robotInUse = this

        waitForGyroCalibration()
    }

    // Driving

    /**
     * Drives the robot forward or backwards for a certain amount of time.
     * @param milliseconds The length of time in seconds to drive.
     * @param power The power to run the motors at when driving.
     */
    fun timeDrive(milliseconds: Long, power: Double = 0.25) {

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
    fun driveToDistanceFromForwardObject(distance: Double, power: Double = 0.175) {

        // Check to see if the opmode is still active.
        if (!linearOpMode.isStopRequested) {
            // Get the current distance for future reference.
            val currentDistance = frontLeftRangeSensor.distanceDetected

            when {
                currentDistance > distance -> {
                    setDrivePower(Math.abs(power))
                    while (frontLeftRangeSensor.distanceDetected > distance && linearOpMode.opModeIsActive()) {
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
    fun driveToDistanceFromLeftObject(distance: Double, power: Double = 0.20) {
        val currentDistance = leftRangeSensor.distanceDetected

        when {
            currentDistance > distance -> {
                setStrafePower(-Math.abs(power))
                while (leftRangeSensor.distanceDetected > distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                setStrafePower(Math.abs(power))
                while (leftRangeSensor.distanceDetected < distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val leftDistance = leftRangeSensor.distanceDetected
        if ((distance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < leftDistance || distance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > leftDistance)
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
    fun driveToDistanceFromRightObject(distance: Double, power: Double = 0.20) {
        val currentDistance = rightRangeSensor.distanceDetected

        when {
            currentDistance > distance -> {
                setStrafePower(Math.abs(power))
                while (rightRangeSensor.distanceDetected > distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            currentDistance < distance -> {
                setStrafePower(-Math.abs(power))
                while (rightRangeSensor.distanceDetected < distance && !linearOpMode.isStopRequested) {
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        linearOpMode.telemetry.update()

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val rightDistance = rightRangeSensor.distanceDetected
        if ((distance + DEFAULT_OBJECT_DISTANCE_TOLERANCE < rightDistance || distance - DEFAULT_OBJECT_DISTANCE_TOLERANCE > rightDistance)
                && !linearOpMode.isStopRequested) {
            driveToDistanceFromRightObject(distance, 0.45)
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot off of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOffBalancingStone(power: Double = 0.175) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            when {
                power > 0 -> {
                    linearOpMode.telemetry.log().add("Driving until offset pitch.")
                    while (startingPitch - pitch < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")

                    linearOpMode.telemetry.log().add("Driving until level pitch.")
                    while (startingPitch - pitch >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")
                }

                power < 0 -> {
                    linearOpMode.telemetry.log().add("Driving until offset pitch.")
                    while (startingPitch - pitch > -BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")

                    linearOpMode.telemetry.log().add("Driving until level pitch.")
                    while (startingPitch - pitch <= -BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    linearOpMode.telemetry.log().add("Detected a pitch of $pitch.")
                }

                else -> return
            }
        }

        stopAllDriveMotors()
    }

    /**
     * Drives the robot on of the balancing stone, using the angle of the robot as an indication of completion.
     * @param power The power ot run the motors at when driving.
     */
    fun driveOnBalancingStone(power: Double = 0.25) {
        if (!linearOpMode.isStopRequested) {
            setDrivePower(power)

            while (abs(startingPitch - pitch) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
                linearOpMode.sleep(10)
            }
        }

        stopAllDriveMotors()
    }

    // Lift

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
     * Sets the lift at a specific location.
     * @param position The position in encoder units that the lift should go to.
     * @param power The power that the lift motor should lift witch motor at.
     */
    fun setLiftHeight(position: LiftPosition, power: Double = 0.5) {
        liftMotor.power = power
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        when(position) {
            LiftPosition.OVEREXTENDED -> liftMotor.targetPosition = 2500
            LiftPosition.FOURTH_LEVEL -> liftMotor.targetPosition = 2000
            LiftPosition.THIRD_LEVEL -> liftMotor.targetPosition = 1500
            LiftPosition.SECOND_LEVEL -> liftMotor.targetPosition = 1000
            LiftPosition.FIRST_LEVEL -> liftMotor.targetPosition = 500
            LiftPosition.BOTTOM_LEVEL -> liftMotor.targetPosition = 0
        }
    }

    /**
     * Lowers the lift until it reaches the bottom.
     */
    fun dropLift() {
        setLiftHeight(LiftPosition.BOTTOM_LEVEL)
    }

    /**
     * Determines if the lift is at the bottom.
     * @return True, if the lift is at the bottom.
     */
    private fun liftIsLowered() = !liftLimitSwitch.state || liftMotor.currentPosition <= -5

    // Glyph Grabbers

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

    // Glyph Deployer

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

    // Jewel Stick

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
        setJewelStickPosition(JEWEL_STICK_UP_POSITION)
        linearOpMode.sleep(delay)
    }

    /**
     * Sets the position of the jewel stick to the lowered position.
     * @param delay The milliseconds to wait after lowering the jewel stick.
     */
    fun lowerJewelStick(delay: Long = 500) {
        setJewelStickPosition(JEWEL_STICK_DOWN_POSITION)
        linearOpMode.sleep(delay)
    }

    // Range Sensors

    /**
     * Begins updating the values of the range sensors.
     */
    private fun startUpdatingRangeSensors() {
        frontLeftRangeSensor.startUpdatingDetectedDistance()
        frontRightRangeSensor.startUpdatingDetectedDistance()
        leftRangeSensor.startUpdatingDetectedDistance()
        rightRangeSensor.startUpdatingDetectedDistance()
        backRangeSensor.startUpdatingDetectedDistance()
    }
}