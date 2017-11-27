package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.libraries.MRIColorBeacon
import java.lang.Math.abs
import kotlin.concurrent.thread

class RelicRecoveryRobot : MecanumRobot() {

    companion object {
        private val FRONT_DISTANCE_SENSOR_FILTER_ALPHA = 0.50
        private val LEFT_DISTANCE_SENSOR_FILTER_ALPHA = 0.50
        private val RIGHT_DISTANCE_SENSOR_FILTER_ALPHA = 0.50
        private val BACK_DISTANCE_SENSOR_FILTER_ALPHA = 0.15

        private val OBJECT_DISTANCE_THRESHOLD = 2.0

        private val GLYPH_GRABBER_OPEN_POSITION = 1.0
        private val GLYPH_GRABBER_RELEASE_POSITION = 0.60
        private val GLYPH_GRABBER_CLOSED_POSITION = 0.40

        private val GLYPH_DEPLOYER_EXTENDED_POSITION = 0.25
        private val GLYPH_DEPLOYER_RETRACTED_POSITION = 0.90

        private val MAXIMUM_ENCODER_LIFT_POSITION = 3000
        val LIFT_FIRST_LEVEL = 300

        private val BALANCING_STONE_ANGLE_THRESHOLD = 2.0
        private val BALANCING_STONE_GROUND_ANGLE_THRESHOLD = 2.0
    }

    private lateinit var liftMotor: DcMotor
    private lateinit var jewelStick: Servo
    private lateinit var leftGlyphGrabber: Servo
    private lateinit var rightGlyphGrabber: Servo
    private lateinit var glyphDeployer: Servo
    private lateinit var jewelColorSensor: ColorSensor
    private lateinit var floorColorSensor: ColorSensor
    private lateinit var leftRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var rightRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var frontRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var backRangeSensor: AnalogInput
    private lateinit var liftLimitSwitch: DigitalChannel
    private lateinit var colorBeacon: MRIColorBeacon

    var frontObjectDistance = 0.0
        private set

    var leftObjectDistance = 0.0
        private set

    var rightObjectDistance = 0.0
        private set

    var backObjectDistance = 0.0
        private set

    // Preparation

    override fun setup(hardwareMap: HardwareMap) {
        frontObjectDistance = 0.0
        leftObjectDistance = 0.0
        rightObjectDistance = 0.0

        liftMotor = hardwareMap.dcMotor.get("winch motor")
        liftMotor.direction = DcMotorSimple.Direction.FORWARD

        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")
        glyphDeployer = hardwareMap.servo.get("glyph deployer")

        jewelColorSensor = hardwareMap.colorSensor.get("color sensor")
        floorColorSensor = hardwareMap.colorSensor.get("floor color sensor")

        frontRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "front range sensor")
        frontRangeSensor.i2cAddress = I2cAddr.create8bit(0x32)

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

    fun start() {
        startUpdatingRangeSensors()
        startUpdatingDriveMotorPowers()
    }

    fun getRobotSetupPosition(hardwareMap: HardwareMap): SetupPosition {
        val colorSensor = hardwareMap.colorSensor.get("floor color sensor")
        val backRangeSensor = hardwareMap.analogInput.get("back range sensor")

        val red = colorSensor.red()
        val blue = colorSensor.blue()
        val backObjectDistance = (backRangeSensor.voltage / (5.0 / 512.0)) * 2.54

        return when {
            red < blue && backObjectDistance > 75 -> SetupPosition.FRONT_BLUE
            red < blue && backObjectDistance < 75 -> SetupPosition.BACK_BLUE
            red > blue && backObjectDistance > 75 -> SetupPosition.FRONT_RED
            red > blue && backObjectDistance < 75 -> SetupPosition.BACK_RED
            else -> SetupPosition.UNKNOWN
        }
    }

    override fun waitForGyroCalibration() {
        super.waitForGyroCalibration()
        colorBeacon.green()
    }

    fun waitForStart() {
        linearOpMode.waitForStart()
        colorBeacon.blue()
    }

    // Driving

    fun timeDrive(milliseconds: Long, power: Double = 0.50) {
        setDrivePower(power)
        linearOpMode.sleep(milliseconds)
        stopAllDriveMotors()
    }

    fun driveToDistanceFromForwardObject(distance: Double, power: Double = 0.15) {
        if(!linearOpMode.isStopRequested) {
            val currentDistance = frontObjectDistance

            when {
                currentDistance > distance -> {
                    setDrivePower(Math.abs(power))
                    while (frontObjectDistance > distance && linearOpMode.opModeIsActive()) {
                        linearOpMode.sleep(10)
                    }
                }

                currentDistance < distance -> { }
            }
        }

        stopAllDriveMotors()
    }

    fun driveToDistanceFromLeftObject(distance: Double, power: Double = 0.50) {
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
        if((distance + OBJECT_DISTANCE_THRESHOLD < leftDistance || distance - OBJECT_DISTANCE_THRESHOLD > leftDistance)
                && linearOpMode.opModeIsActive()) {
            driveToDistanceFromLeftObject(distance, 0.45)
        }

        stopAllDriveMotors()
    }

    fun driveToDistanceFromRightObject(distance: Double, power: Double = 0.50) {
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
                    linearOpMode.telemetry.addData("Distance", rightObjectDistance)
                    linearOpMode.telemetry.update()
                    linearOpMode.sleep(10)
                }
            }

            else -> return
        }

        linearOpMode.telemetry.update()

        stopAllDriveMotors()
        linearOpMode.sleep(1000)

        val rightDistance = rightObjectDistance
        if((distance + OBJECT_DISTANCE_THRESHOLD < rightDistance || distance - OBJECT_DISTANCE_THRESHOLD > rightDistance)
                && !linearOpMode.isStopRequested) {
            driveToDistanceFromRightObject(distance, 0.45)
        }

        stopAllDriveMotors()
    }

    fun driveOffBalancingStone(power: Double = 0.15) {
        val startingPitch = getPitch()
        setDrivePower(power)

        while(abs(startingPitch - getPitch()) < BALANCING_STONE_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
            linearOpMode.telemetry.addData("Pitch", getPitch())
            linearOpMode.telemetry.update()
            linearOpMode.sleep(10)
        }

        while(abs(startingPitch - getPitch()) >= BALANCING_STONE_GROUND_ANGLE_THRESHOLD && !linearOpMode.isStopRequested) {
            linearOpMode.telemetry.addData("Pitch", getPitch())
            linearOpMode.telemetry.update()
            linearOpMode.sleep(10)
        }

        stopAllDriveMotors()
    }

    // Lift

    fun setLiftWinchPower(power: Double) {
        if ((power >= 0 && liftMotor.currentPosition <= MAXIMUM_ENCODER_LIFT_POSITION) || (!liftIsLowered() && power < 0))
            liftMotor.power = power
        else
            liftMotor.power = 0.0
    }

    fun setLiftPosition(position: Int, power: Double = 0.30) {
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

        setLiftWinchPower(0.0)
    }

    fun dropLift(power: Double = 0.30) {
        if(!linearOpMode.isStopRequested) {
            setLiftWinchPower(-Math.abs(power))
            while (linearOpMode.opModeIsActive() && !liftIsLowered()) {
                linearOpMode.sleep(10)
            }

            setLiftWinchPower(0.0)
            liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    private fun liftIsLowered() = liftLimitSwitch.state

    // Glyph Grabbers

    private fun setGlyphGrabbersPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = 1 - position
    }

    fun openGlyphGrabbers() { setGlyphGrabbersPosition(GLYPH_GRABBER_OPEN_POSITION) }

    fun releaseGlyphGrabbers() { setGlyphGrabbersPosition(GLYPH_GRABBER_RELEASE_POSITION)}

    fun closeGlyphGrabbers() { setGlyphGrabbersPosition(GLYPH_GRABBER_CLOSED_POSITION) }

    // Glyph Deployer

    fun extendGlyphDeployer() { glyphDeployer.position = GLYPH_DEPLOYER_EXTENDED_POSITION }

    fun retractGlyphDeployer() { glyphDeployer.position = GLYPH_DEPLOYER_RETRACTED_POSITION }

    // Jewel Stick

    private fun setJewelStickPosition(position: Double) {
        jewelStick.position = position
    }

    fun raiseJewelStick() {
        setJewelStickPosition(0.0)
    }

    // Range Sensors

    private fun startUpdatingRangeSensors() {
        thread(start = true) {
            while(!linearOpMode.isStopRequested) {
                updateFrontObjectDistance()
                updateLeftObjectDistance()
                updateRightObjectDistance()
                updateBackObjectDistance()
                Thread.sleep(10)
            }
        }
    }

    private fun updateFrontObjectDistance() {
        val rawDistance = frontRangeSensor.cmUltrasonic()
        frontObjectDistance += FRONT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - frontObjectDistance)
    }

    private fun updateLeftObjectDistance() {
        val rawDistance = leftRangeSensor.cmUltrasonic()
        leftObjectDistance += LEFT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - leftObjectDistance)
    }

    private fun updateRightObjectDistance() {
        val rawDistance = rightRangeSensor.cmUltrasonic()
        rightObjectDistance += RIGHT_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - rightObjectDistance)
    }

    private fun updateBackObjectDistance() {
        val rawDistance = (backRangeSensor.voltage / (5.0 / 512.0)) * 2.54
        backObjectDistance += BACK_DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - backObjectDistance)
    }

    // Teleop

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

    // Self Driving

    fun deliverGlyph() {
        val frontObjectDistance = frontObjectDistance
        val leftObjectDistance = leftObjectDistance
        val rightObjectDistance = rightObjectDistance
        val heading = getHeading()

        when(heading) {
            in -45..45 -> {
                when {
                    leftObjectDistance < rightObjectDistance -> {}
                    rightObjectDistance < leftObjectDistance -> {}
                }
            }

            in -135..-45 -> {
                when {
                    leftObjectDistance < rightObjectDistance -> {}
                    rightObjectDistance < leftObjectDistance -> {}
                }
            }

            in 45..135 -> {
                when {
                    leftObjectDistance < rightObjectDistance -> {}
                    rightObjectDistance < leftObjectDistance -> {}
                }
            }
        }
    }
}