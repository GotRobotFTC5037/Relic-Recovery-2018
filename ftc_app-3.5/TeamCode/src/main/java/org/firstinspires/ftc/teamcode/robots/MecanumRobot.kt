package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference

open class MecanumRobot : Robot() {

    private lateinit var frontLeftMotor: DcMotor
    private lateinit var frontRightMotor: DcMotor
    private lateinit var backLeftMotor: DcMotor
    private lateinit var backRightMotor: DcMotor
    private lateinit var imu: BNO055IMU

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
        calibration.dxAccel = 5; calibration.dyAccel = 10; calibration.dzAccel = 39
        calibration.dxGyro  = 0; calibration.dyGyro  =  0; calibration.dzGyro  = -1
        calibration.dxMag   = 0; calibration.dyMag   =  0; calibration.dzMag   =  0
        calibration.radiusAccel = 1000; calibration.radiusMag = 480

        val imuParameters = BNO055IMU.Parameters()
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imuParameters.calibrationData = calibration

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(imuParameters)
    }

    override fun setDrivePower(power: Double) {
        frontLeftMotor.power = power
        backLeftMotor.power = power
        frontRightMotor.power = power
        backRightMotor.power = power
    }

    override fun setTurnPower(power: Double) {
        frontLeftMotor.power = -power
        backLeftMotor.power = -power
        frontRightMotor.power = power
        backRightMotor.power = power
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

    fun getHeading() : Double {
        val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
        return orientation.firstAngle.toDouble()
    }

    override fun turn(power: Double, degrees: Int) {
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

    fun waitForGyroCalibration() {
        while (!imu.isGyroCalibrated && linearOpMode.opModeIsActive()) {
            linearOpMode.sleep(100)
        }
    }
}

