package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*

import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier

object RelicRecoveryRobot : MecanumRobot() {

    val MIN_LIFT_POSITION = 0
    val LIFT_STEP_ONE = 25
    val LIFT_STEP_TWO = 50
    val LIFT_STEP_THREE = 75
    val LIFT_STEP_FOUR = 100

    private lateinit var mWinchMotor: DcMotor

    private lateinit var mJewelStick: Servo
    private lateinit var mLeftGlyphGrabber: Servo
    private lateinit var mRightGlyphGrabber: Servo

    lateinit var jewelColorSensor: ColorSensor
    private lateinit var positionColorSensor: ColorSensor

    // TODO: Implement these sensors.
    lateinit var rangeSensor: ModernRoboticsI2cRangeSensor
    lateinit var liftLimitSwitch: DigitalChannel

    private val pictographIdentifier = PictographIdentifier()

    override fun setup(linearOpMode: LinearOpMode) {
        super.setup(linearOpMode)

        val hardwareMap = linearOpMode.hardwareMap

        mWinchMotor = hardwareMap.dcMotor.get("winch motor")
        mWinchMotor.direction = DcMotorSimple.Direction.REVERSE

        mJewelStick = hardwareMap.servo.get("jewel stick")
        mLeftGlyphGrabber = hardwareMap.servo.get("left grabber")
        mRightGlyphGrabber = hardwareMap.servo.get("right grabber")

        jewelColorSensor = hardwareMap.colorSensor.get("color sensor")
        positionColorSensor = hardwareMap.colorSensor.get("position color sensor")

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "range sensor");

        liftLimitSwitch = hardwareMap.digitalChannel.get("limit switch")
        liftLimitSwitch.mode = DigitalChannel.Mode.INPUT

        pictographIdentifier.activate()

        isSetup = true
    }

    private fun setGlyphGrabbersPosition(position: Double) {
        mLeftGlyphGrabber.position = position
        mRightGlyphGrabber.position = 1 - position
    }

    fun openGlyphGrabbers() {
        setGlyphGrabbersPosition(0.25)
    }

    fun closeGlyphGrabbers() {
        setGlyphGrabbersPosition(0.75)
    }

    fun setWinchPower(power: Double) {
        mWinchMotor.power = power
    }

    private fun setJewelStickPosition(position: Double) {
        mJewelStick.position = position
    }

    fun lowerJewelStick() {
        setJewelStickPosition(0.90)
    }

    fun raiseJewelStick() {
        setJewelStickPosition(0.0)
    }

    fun getAllianceColor(): AllianceColor {
        val red = positionColorSensor.red()
        val blue = positionColorSensor.blue()

        return when {
            red > blue -> AllianceColor.RED
            red < blue -> AllianceColor.BLUE
            else -> AllianceColor.UNKNOWN
        }
    }

    fun getOpticalDistanceValue(): Int {
        return rangeSensor.rawOptical()
    }

    fun getUltrasonicDistanceValue(): Double {
        linearOpMode.telemetry.addLine(rangeSensor.cmUltrasonic().toString())
        linearOpMode.telemetry.update()
        return rangeSensor.cmUltrasonic()
    }

    fun driveUntilThresholdReached(threshold: Double, power: Double) {
        while (getUltrasonicDistanceValue() > threshold && linearOpMode.opModeIsActive()) {
            setDrivePower(power)
        }
        stopAllDriveMotors()
    }

}