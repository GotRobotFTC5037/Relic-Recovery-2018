package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.libraries.MRIColorBeacon

object RelicRecoveryRobot : MecanumRobot() {

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

    override fun setup(hardwareMap: HardwareMap) {
        liftMotor = hardwareMap.dcMotor.get("winch motor")
        liftMotor.direction = DcMotorSimple.Direction.FORWARD

        jewelStick = hardwareMap.servo.get("jewel stick")
        leftGlyphGrabber = hardwareMap.servo.get("left grabber")
        rightGlyphGrabber = hardwareMap.servo.get("right grabber")

        jewelColorSensor = hardwareMap.colorSensor.get("color sensor")
        floorColorSensor = hardwareMap.colorSensor.get("floor color sensor")

        frontUltrasonicSensor = hardwareMap.analogInput.get("front ultrasonic sensor")
        leftRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "left range sensor");
        rightRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, "right range sensor")
        rightRangeSensor.i2cAddress = I2cAddr.create8bit(0x46)

        liftLimitSwitch = hardwareMap.digitalChannel.get("limit switch")
        liftLimitSwitch.mode = DigitalChannel.Mode.INPUT

        colorBeacon = MRIColorBeacon()
        colorBeacon.init(hardwareMap, "color beacon")
        colorBeacon.yellow()

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

    fun driveToDistanceFromForwardObject(distance: Double, power: Double) {
        setDrivePower(power)
        while (getFrontObjectDistance() > distance && linearOpMode.opModeIsActive()) {
            linearOpMode.sleep(50)
        }
        stopAllDriveMotors()
    }

    fun setLiftWinchPower(power: Double) {
        if((power >= 0 && liftMotor.currentPosition <= 2650) || (!liftIsLowered() && power < 0))
            liftMotor.power = power
        else liftMotor.power = 0.0

        if(liftIsLowered()) {
            liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

    }

    fun setLiftPosition(power: Double, position: Int) {
        if(liftMotor.currentPosition < position) {
            setLiftWinchPower(Math.abs(power))
            while (liftMotor.currentPosition < position && linearOpMode.opModeIsActive()) {
                linearOpMode.sleep(50)
            }
        } else if(liftMotor.currentPosition > position) {
            setLiftWinchPower(-Math.abs(power))
            while (liftMotor.currentPosition > position && linearOpMode.opModeIsActive()) {
                linearOpMode.sleep(50)
            }
        }
    }

    private fun setGlyphGrabbersPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = 1 - position
    }
    fun openGlyphGrabbers() { setGlyphGrabbersPosition(0.25) }
    fun closeGlyphGrabbers() { setGlyphGrabbersPosition(0.75) }

    private fun setJewelStickPosition(position: Double) { jewelStick.position = position }
    fun lowerJewelStick() { setJewelStickPosition(0.85) }
    fun raiseJewelStick() { setJewelStickPosition(0.0) }

    private fun getFrontObjectDistance() = (frontUltrasonicSensor.voltage / (5.0 / 512.0)) * 2.54
    fun getLeftObjectDistance() = leftRangeSensor.cmUltrasonic()
    fun getRightObjectDistance() = rightRangeSensor.cmUltrasonic()

    private fun liftIsLowered() = liftLimitSwitch.state

    // TODO: Add a method that keeps the robot locked onto the balancing stone.

    // TODO: Add heading correction to drive method.

    // TODO: Add vector correction to drive method.

    // TODO: Add a way to lock position from a front object while strafing.

    // TODO: Add an automated way to set lift positions.

    // TODO: Create an integrating accelerometer algorithm to detect position.

    // TODO: Add slowdown as target turn position or drive distance is reached.

    // TODO: Add drive functions that use the integrating accelerometer to detect distance.
}