package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.libraries.RobotAccelerationIntegrator
import kotlin.concurrent.thread

open class MecanumRobot : Robot() {

    private lateinit var frontLeftMotor: DcMotor
    private lateinit var frontRightMotor: DcMotor
    private lateinit var backLeftMotor: DcMotor
    private lateinit var backRightMotor: DcMotor
    lateinit var imu: BNO055IMU

    private var frontLeftMotorPower = 0.0
    private var frontRightMotorPower = 0.0
    private var backLeftMotorPower = 0.0
    private var backRightMotorPower = 0.0

    private var targetHeading = 0.0

    var shouldCorrectHeading = false

    // Preparing

    override fun setup(hardwareMap: HardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("front left motor")
        frontLeftMotor.direction = DcMotorSimple.Direction.FORWARD
        frontLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        frontRightMotor = hardwareMap.dcMotor.get("front right motor")
        frontRightMotor.direction = DcMotorSimple.Direction.REVERSE
        frontRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backLeftMotor = hardwareMap.dcMotor.get("back left motor")
        backLeftMotor.direction = DcMotorSimple.Direction.FORWARD
        backLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backRightMotor = hardwareMap.dcMotor.get("back right motor")
        backRightMotor.direction = DcMotorSimple.Direction.REVERSE
        backRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val calibration = BNO055IMU.CalibrationData()
        calibration.dxAccel = -19; calibration.dyAccel = -33; calibration.dzAccel = 19
        calibration.dxGyro = -2; calibration.dyGyro = -1; calibration.dzGyro = 1
        calibration.dxMag = 0; calibration.dyMag = 0; calibration.dzMag =  0
        calibration.radiusAccel = 1000; calibration.radiusMag = 480

        val imuParameters = BNO055IMU.Parameters()
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imuParameters.calibrationData = calibration
        imuParameters.accelerationIntegrationAlgorithm = RobotAccelerationIntegrator()

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(imuParameters)
    }

    // Moving

    override fun setDrivePower(power: Double) {
        frontLeftMotorPower = power
        frontRightMotorPower = power
        backLeftMotorPower = power
        backRightMotorPower = power
    }

    override fun setTurnPower(power: Double) {
        frontLeftMotorPower = -power
        frontRightMotorPower = power
        backLeftMotorPower = -power
        backRightMotorPower = power
    }

    fun setStrafePower(power: Double) {
        frontLeftMotorPower = power
        frontRightMotorPower = -power
        backLeftMotorPower = -power
        backRightMotorPower = power
    }

    override fun stopAllDriveMotors() {
        frontLeftMotorPower = 0.0
        frontRightMotorPower = 0.0
        backLeftMotorPower = 0.0
        backRightMotorPower = 0.0
    }

    fun setDirection(x: Double, y: Double, z: Double) {
        val gyroHeading = Math.toRadians(getHeading() + 90.0)
        val sin = Math.sin(gyroHeading)
        val cos = Math.cos(gyroHeading)
        var adjustedX = (sin * x) - (cos * y)
        var adjustedY = (sin * y) + (cos * x)
        var adjustedZ = z

        val max = Math.abs(adjustedY) + Math.abs(adjustedX) + Math.abs(adjustedZ)
        if(max > 1.0) {
            adjustedX /= max
            adjustedY /= max
            adjustedZ /= max
        }

        frontLeftMotor.power =  adjustedY + adjustedX - adjustedZ
        backLeftMotor.power = adjustedY - adjustedX - adjustedZ
        frontRightMotor.power = adjustedY - adjustedX + adjustedZ
        backRightMotor.power = adjustedY + adjustedX + adjustedZ
    }

    override fun turn(power: Double, degrees: Double) {
        shouldCorrectHeading = false
        targetHeading += degrees

        if (degrees > 0) {
            val targetHeading = getHeading() + degrees
            setTurnPower(Math.abs(power))
            while (getHeading() < targetHeading && linearOpMode.opModeIsActive()) {
                linearOpMode.sleep(50)
            }
        } else if (degrees < 0) {
            val targetHeading = getHeading() + degrees
            setTurnPower(-Math.abs(power))
            while (getHeading() > targetHeading && linearOpMode.opModeIsActive()) {
               linearOpMode.sleep(50)
            }
        }

        stopAllDriveMotors()
        shouldCorrectHeading = true
    }

    fun startUpdatingDriveMotorPowers() {
        linearOpMode.waitForStart()
        thread(start = true, priority = 7) {
            while(linearOpMode.opModeIsActive()) {
                val headingCorrection = if(shouldCorrectHeading) headingCorrection() else 0.0
                frontLeftMotor.power = Range.clip(frontLeftMotorPower - headingCorrection, -1.0, 1.0)
                frontRightMotor.power = Range.clip(frontRightMotorPower + headingCorrection, -1.0, 1.0)
                backLeftMotor.power = Range.clip(backLeftMotorPower - headingCorrection, -1.0, 1.0)
                backRightMotor.power = Range.clip(backRightMotorPower + headingCorrection, -1.0, 1.0)
                Thread.yield()
            }
        }
    }

    // IMU

    private fun getHeading(): Double {
        val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        return orientation.firstAngle.toDouble()
    }

    fun waitForGyroCalibration() {
        while (!imu.isGyroCalibrated && linearOpMode.opModeIsActive()) {
            linearOpMode.sleep(100)
        }
    }

    private fun headingCorrection(): Double = (targetHeading - getHeading()) * GYRO_HEADING_COEFFICIENT

    // Constants

    companion object {
        val MINIMUM_DRIVE_POWER = 0.10
        val GYRO_HEADING_COEFFICIENT = 0.015
    }
}

