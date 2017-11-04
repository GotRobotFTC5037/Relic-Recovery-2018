package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.*

import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier

class RelicRecoveryRobot : MecanumRobot() {
    private lateinit var mWinchMotor: DcMotor

    private lateinit var mJewelStick: Servo
    private lateinit var mLeftGlyphGrabber: Servo
    private lateinit var mRightGlyphGrabber: Servo

    // TODO: Implement these sensors.
    lateinit var liftLimitSwitch: DigitalChannel
    lateinit var colorSensor: ColorSensor
    lateinit var leftRangeSensor: ModernRoboticsI2cRangeSensor
    lateinit var rightRangeSensor: ModernRoboticsI2cRangeSensor

    private val pictographIdentifier = PictographIdentifier()

    override fun setup(linearOpMode: LinearOpMode) {
        super.setup(linearOpMode)

        mWinchMotor = linearOpMode.hardwareMap.dcMotor.get("winch motor")
        mWinchMotor.direction = DcMotorSimple.Direction.FORWARD

        mJewelStick = linearOpMode.hardwareMap.servo.get("jewel stick")
        mLeftGlyphGrabber = linearOpMode.hardwareMap.servo.get("left grabber")
        mRightGlyphGrabber = linearOpMode.hardwareMap.servo.get("right grabber")

        colorSensor = linearOpMode.hardwareMap.colorSensor.get("color sensor")

        pictographIdentifier.activate()
    }

    private fun setGlyphGrabbersPosition(position: Double) {
        mLeftGlyphGrabber.position = position
        mRightGlyphGrabber.position = 1 - position
    }

    fun openGlyphGrabbers() {
        setGlyphGrabbersPosition(1.0)
    }

    fun closeGlyphGrabbers() {
        setGlyphGrabbersPosition(0.0)
    }

    fun setWinchPower(power: Double) {
        mWinchMotor.power = power
    }

    private fun setJewelStickPosition(position: Double) {
        mJewelStick.position = position
    }

    fun lowerJewelStick() {
        setJewelStickPosition(1.0)
    }

    fun raiseJewelStick() {
        setJewelStickPosition(0.0)
    }
}
