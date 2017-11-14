package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.libraries.MRIColorBeacon
import kotlin.concurrent.thread

object RelicRecoveryRobot : MecanumRobot() {

    private val MAXIMUM_ENCODER_LIFT_POSITION = 2700
    private val DISTANCE_SENSOR_FILTER_ALPHA = 0.15

    private val LIFT_CORRECTION_COEFFICIENT = 0.002

    val LIFT_FIRST_LEVEL = 350
    val LIFT_SECOND_LEVEL = 700
    val LIFT_THIRD_LEVEL = 1050
    val LIFT_FORTH_LEVEL = 1400
    val LIFT_FIFTH_LEVEL = 2700


    private lateinit var liftMotor: DcMotor
    private lateinit var jewelStick: Servo
    private lateinit var leftGlyphGrabber: Servo
    private lateinit var rightGlyphGrabber: Servo
    private lateinit var jewelColorSensor: ColorSensor
    private lateinit var floorColorSensor: ColorSensor
    private lateinit var leftRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var rightRangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var frontUltrasonicSensor: AnalogInput
    private lateinit var liftLimitSwitch: DigitalChannel
    lateinit var colorBeacon: MRIColorBeacon

    private var targetLiftPosition = 0
    private var shouldHoldLiftPosition = true

    private var frontObjectDistance = 0.0
    private var leftObjectDistance = 0.0
    private var rightObjectDistance = 0.0

    // Preparation

    override fun setup(hardwareMap: HardwareMap) {
        liftMotor = hardwareMap.dcMotor.get("winch motor")
        liftMotor.direction = DcMotorSimple.Direction.FORWARD

        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")

        jewelColorSensor = hardwareMap.colorSensor.get("color sensor")
        floorColorSensor = hardwareMap.colorSensor.get("floor color sensor")

        frontUltrasonicSensor = hardwareMap.analogInput.get("front ultrasonic sensor")
        leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "left range sensor")
        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "right range sensor")
        rightRangeSensor.i2cAddress = I2cAddr.create8bit(0x46)

        liftLimitSwitch = hardwareMap.digitalChannel.get("limit switch")
        liftLimitSwitch.mode = DigitalChannel.Mode.INPUT

        colorBeacon = MRIColorBeacon()
        colorBeacon.init(hardwareMap, "color beacon")
        colorBeacon.yellow()

        beginHoldingLiftPosition()

        thread(start = true, priority = 7) {
            linearOpMode.waitForStart()
            while(linearOpMode.opModeIsActive()) {
                updateFrontObjectDistance()
                updateLeftObjectDistance()
                updateRightObjectDistance()
                Thread.sleep(20)
            }
        }

        super.setup(hardwareMap)
    }

    fun getAllianceColor(hardwareMap: HardwareMap): AllianceColor {
        val colorSensor = hardwareMap.colorSensor.get("floor color sensor")
        val red = colorSensor.red()
        val blue = colorSensor.blue()

        return when {
            red > blue -> AllianceColor.RED
            red < blue -> AllianceColor.BLUE
            else -> AllianceColor.UNKNOWN
        }
    }

    // Driving

    fun driveToDistanceFromForwardObject(distance: Double, power: Double) {
        val currentDistance = frontObjectDistance
        when {
            currentDistance > distance -> {
                setDrivePower(Math.abs(power))
                while (frontObjectDistance > distance && linearOpMode.opModeIsActive()) {
                    linearOpMode.telemetry.addData("Front Distance", updateFrontObjectDistance())
                    linearOpMode.telemetry.addData("Target", distance)
                    linearOpMode.telemetry.update()
                    linearOpMode.sleep(50)
                }
            }

            currentDistance < distance -> {
                setDrivePower(-Math.abs(power))
                while (frontObjectDistance < distance && linearOpMode.opModeIsActive()) {
                    linearOpMode.sleep(50)
                }
            }

            else -> return
        }

        stopAllDriveMotors()
    }

    fun driveToDistanceFromLeftObject(distance: Double, power: Double) {
        val currentDistance = leftObjectDistance
        when {
            currentDistance > distance -> {
                setStrafePower(-Math.abs(power))
                while (leftObjectDistance > distance && linearOpMode.opModeIsActive()) {
                    linearOpMode.sleep(50)
                }
            }

            currentDistance < distance -> {
                setStrafePower(Math.abs(power))
                while (leftObjectDistance < distance && linearOpMode.opModeIsActive()) {
                    linearOpMode.telemetry.addData("Left Distance", updateLeftObjectDistance())
                    linearOpMode.telemetry.addData("Target", distance)
                    linearOpMode.telemetry.update()
                    linearOpMode.sleep(50)
                }
            }

            else -> return
        }

        stopAllDriveMotors()
    }

    // Lift

    fun setLiftWinchPower(power: Double) {
        if (power == 0.0) {
            shouldHoldLiftPosition = true;
            targetLiftPosition = liftMotor.currentPosition
        } else {
            shouldHoldLiftPosition = false
        }

        if ((power >= 0 && liftMotor.currentPosition <= MAXIMUM_ENCODER_LIFT_POSITION) || (!liftIsLowered() && power < 0))
            liftMotor.power = power
        else
            liftMotor.power = 0.0

        if (liftIsLowered()) {
            liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    fun setLiftPosition(position: Int, power: Double = 0.1) {
        shouldHoldLiftPosition = false

        when {
            liftMotor.currentPosition < position -> {
                setLiftWinchPower(Math.abs(power))
                while (liftMotor.currentPosition < position && linearOpMode.opModeIsActive()) {
                    if (liftIsLowered()) {
                        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    }
                    linearOpMode.sleep(50)
                }
            }

            liftMotor.currentPosition > position -> {
                setLiftWinchPower(-Math.abs(power))
                while (liftMotor.currentPosition > position && linearOpMode.opModeIsActive()) {
                    if (liftIsLowered()) {
                        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    }
                    linearOpMode.sleep(50)
                }
            }
        }

        setLiftWinchPower(0.0)
        targetLiftPosition = position
        shouldHoldLiftPosition = true
    }

    fun liftIsLowered() = liftLimitSwitch.state

    private fun beginHoldingLiftPosition() {
        thread(true) {
            linearOpMode.waitForStart()
            while (linearOpMode.opModeIsActive()) {
                if(shouldHoldLiftPosition) {
                    val liftPowerAdjustment = (targetLiftPosition - liftMotor.currentPosition) * LIFT_CORRECTION_COEFFICIENT
                    liftMotor.power = liftPowerAdjustment
                }
                Thread.sleep(100)
            }
        }
    }

    // Glyph Grabbers

    private fun setGlyphGrabbersPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = 1 - position
    }

    fun openGlyphGrabbers() {
        setGlyphGrabbersPosition(0.25)
    }

    fun closeGlyphGrabbers() {
        setGlyphGrabbersPosition(0.775)
    }

    // Jewel Stick

    private fun setJewelStickPosition(position: Double) {
        jewelStick.position = position
    }

    fun lowerJewelStick() {
        setJewelStickPosition(0.85)
    }

    fun raiseJewelStick() {
        setJewelStickPosition(0.0)
    }

    // Range Sensors

    private fun updateFrontObjectDistance() {
        val rawDistance = (frontUltrasonicSensor.voltage / (5.0 / 512.0)) * 2.54
        if(frontObjectDistance == 0.0) { frontObjectDistance = rawDistance; return }
        frontObjectDistance += DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - frontObjectDistance)
    }

    private fun updateLeftObjectDistance() {
        val rawDistance = leftRangeSensor.cmUltrasonic()
        if(leftObjectDistance == 0.0) { leftObjectDistance = rawDistance; return }
        leftObjectDistance += DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - leftObjectDistance)
    }

    private fun updateRightObjectDistance() {
        val rawDistance = rightRangeSensor.cmUltrasonic()
        if(rightObjectDistance == 0.0) { rightObjectDistance = rawDistance; return }
        rightObjectDistance += DISTANCE_SENSOR_FILTER_ALPHA * (rawDistance - rightObjectDistance)
    }
}