package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.GyroSensor
import com.qualcomm.robotcore.hardware.HardwareMap

open class MecanumRobot : Robot() {

    private lateinit var frontLeftMotor: DcMotor
    private lateinit var frontRightMotor: DcMotor
    private lateinit var backLeftMotor: DcMotor
    private lateinit var backRightMotor: DcMotor
    private lateinit var gyroSensor: GyroSensor

    private var lastRawGyroHeading = 0
    private var gyroHeading = 0

    override fun setup(hardwareMap: HardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("front left motor")
        frontLeftMotor.direction = DcMotorSimple.Direction.FORWARD

        frontRightMotor = hardwareMap.dcMotor.get("front right motor")
        frontRightMotor.direction = DcMotorSimple.Direction.REVERSE

        backLeftMotor = hardwareMap.dcMotor.get("back left motor")
        backLeftMotor.direction = DcMotorSimple.Direction.FORWARD

        backRightMotor = hardwareMap.dcMotor.get("back right motor")
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE

        gyroSensor = hardwareMap.gyroSensor.get("gyro")
        gyroSensor.calibrate()
    }

    override fun setDrivePower(power: Double) {
        frontLeftMotor.power = power
        backLeftMotor.power = power
        frontRightMotor.power = power
        backRightMotor.power = power
    }

    override fun setTurnPower(power: Double) {
        frontLeftMotor.power = power
        backLeftMotor.power = power
        frontRightMotor.power = -power
        backRightMotor.power = -power
    }

    override fun stopAllDriveMotors() {
        frontLeftMotor.power = 0.0
        backLeftMotor.power = 0.0
        frontRightMotor.power = 0.0
        backRightMotor.power = 0.0
    }

    fun setStrafePower(power: Double) {
        frontLeftMotor.power = power
        frontRightMotor.power = -power
        backLeftMotor.power = -power
        backRightMotor.power = power
    }

    override fun turn(power: Double, degrees: Int) {
        if (degrees > 0) {
            val targetHeading = getGyroHeading() + degrees
            setTurnPower(Math.abs(power))
            while (getGyroHeading() < targetHeading && linearOpMode.opModeIsActive()) {
                linearOpMode.telemetry.addData("Heading", getGyroHeading())
                linearOpMode.telemetry.update()
                linearOpMode.idle()
            }
        } else if (degrees < 0) {
            val targetHeading = getGyroHeading() + degrees
            setTurnPower(-Math.abs(power))
            while (getGyroHeading() > targetHeading && linearOpMode.opModeIsActive()) {
                linearOpMode.telemetry.addData("-Heading", getGyroHeading())
                linearOpMode.telemetry.update()
                linearOpMode.idle()
            }
        }

        stopAllDriveMotors()
    }

    fun setDirection(x: Double, y: Double, z: Double) {
        val wheelSet1ComponentPower = (y - x) / 2
        val wheelSet2ComponentPower = (y + x) / 2

        val wheelSet1Power = Math.sqrt(2 * Math.pow(wheelSet1ComponentPower, 2.0)) * Math.signum(wheelSet1ComponentPower)
        val wheelSet2Power = Math.sqrt(2 * Math.pow(wheelSet2ComponentPower, 2.0)) * Math.signum(wheelSet2ComponentPower)

        frontRightMotor.power = wheelSet1Power - z
        backLeftMotor.power = wheelSet1Power + z
        frontLeftMotor.power = wheelSet2Power + z
        backRightMotor.power = wheelSet2Power - z
    }

    fun waitForGyroCalibration() {
        while (gyroSensor.isCalibrating && linearOpMode.opModeIsActive()) {
            linearOpMode.sleep(50)
        }
    }

    fun getGyroHeading(): Double {
        val rawGyroHeading = gyroSensor.heading

        gyroHeading = when {
            rawGyroHeading - 180 > lastRawGyroHeading -> gyroHeading + rawGyroHeading - 360 - lastRawGyroHeading
            rawGyroHeading + 180 < lastRawGyroHeading -> gyroHeading + rawGyroHeading + 360 - lastRawGyroHeading
            else -> gyroHeading + rawGyroHeading - lastRawGyroHeading
        }

        lastRawGyroHeading = rawGyroHeading
        return (-gyroHeading).toDouble()
    }
}

