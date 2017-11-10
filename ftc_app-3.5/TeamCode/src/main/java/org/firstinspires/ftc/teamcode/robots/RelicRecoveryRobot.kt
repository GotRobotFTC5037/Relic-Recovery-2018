package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.hardware.*

object RelicRecoveryRobot : MecanumRobot() {

    const val MIN_LIFT_POSITION = 0
    const val LIFT_STEP_ONE = 25
    const val LIFT_STEP_TWO = 50
    const val LIFT_STEP_THREE = 75
    const val LIFT_STEP_FOUR = 100

    private lateinit var winchMotor: DcMotor
    private lateinit var jewelStick: Servo
    private lateinit var leftGlyphGrabber: Servo
    private lateinit var rightGlyphGrabber: Servo
    private lateinit var jewelColorSensor: ColorSensor
    private lateinit var positionColorSensor: ColorSensor
    private lateinit var rangeSensor: ModernRoboticsI2cRangeSensor
    private lateinit var liftLimitSwitch: DigitalChannel

    private val ultrasonicDistanceSensorValue get() = rangeSensor.cmUltrasonic()

    val opticalDistanceSensorValue get() = rangeSensor.cmOptical()

    override fun setup(hardwareMap: HardwareMap) {
        winchMotor = hardwareMap.dcMotor.get("winch motor")
        winchMotor.direction = DcMotorSimple.Direction.REVERSE
        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")
        jewelColorSensor = hardwareMap.colorSensor.get("color sensor")
        positionColorSensor = hardwareMap.colorSensor.get("position color sensor")
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "range sensor");
        liftLimitSwitch = hardwareMap.digitalChannel.get("limit switch")
        liftLimitSwitch.mode = DigitalChannel.Mode.INPUT

        super.setup(hardwareMap)
    }

    fun getAllianceColor(hardwareMap: HardwareMap): AllianceColor {
        val colorSensor = hardwareMap.colorSensor.get("position color sensor")
        val red = colorSensor.red()
        val blue = colorSensor.blue()

        return when {
            red > blue -> AllianceColor.RED
            red < blue -> AllianceColor.BLUE
            else -> AllianceColor.UNKNOWN
        }
    }

    fun driveUntilThresholdReached(threshold: Double, power: Double) {
        while (ultrasonicDistanceSensorValue > threshold && linearOpMode.opModeIsActive()) {
            setDrivePower(power)
        }
        stopAllDriveMotors()
    }

    fun setWinchPower(power: Double) {
        winchMotor.power = power
    }

    private fun setGlyphGrabbersPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = 1 - position
    }

    fun openGlyphGrabbers() {
        setGlyphGrabbersPosition(0.25)
    }

    fun closeGlyphGrabbers() {
        setGlyphGrabbersPosition(0.75)
    }

    private fun setJewelStickPosition(position: Double) {
        jewelStick.position = position
    }

    fun lowerJewelStick() {
        setJewelStickPosition(0.90)
    }

    fun raiseJewelStick() {
        setJewelStickPosition(0.0)
    }
}