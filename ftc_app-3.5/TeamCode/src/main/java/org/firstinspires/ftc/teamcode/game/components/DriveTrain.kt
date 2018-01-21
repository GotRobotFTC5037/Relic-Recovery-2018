package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.libraries.PIDController
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrain.Heading
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrain.MecanumDriveTrain

class DriveTrain(linearOpMode: LinearOpMode) : MecanumDriveTrain(linearOpMode) {

    override val frontLeftMotor: DcMotor
        get() {
            val motor = hardwareMap.dcMotor.get("front left motor")
            motor.direction = DcMotorSimple.Direction.FORWARD
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            return motor
        }

    override val frontRightMotor: DcMotor
        get() {
            val motor = hardwareMap.dcMotor.get("right left motor")
            motor.direction = DcMotorSimple.Direction.FORWARD
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            return motor
        }

    override val rearLeftMotor: DcMotor
        get() {
            val motor = hardwareMap.dcMotor.get("back left motor")
            motor.direction = DcMotorSimple.Direction.FORWARD
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            return motor
        }

    override val rearRightMotor: DcMotor
        get() {
            val motor = hardwareMap.dcMotor.get("back left motor")
            motor.direction = DcMotorSimple.Direction.FORWARD
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            return motor
        }

    private val imu: BNO055IMU by lazy {
        val calibration = BNO055IMU.CalibrationData()
        calibration.dxAccel = -19; calibration.dyAccel = -33; calibration.dzAccel = 19
        calibration.dxGyro = -2; calibration.dyGyro = -1; calibration.dzGyro = 1
        calibration.dxMag = 0; calibration.dyMag = 0; calibration.dzMag =  0
        calibration.radiusAccel = 1000; calibration.radiusMag = 480

        val imuParameters = BNO055IMU.Parameters()
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imuParameters.calibrationData = calibration

        val imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(imuParameters)

        imu
    }

    fun waitForGyroCalibration() {
        while (!imu.isGyroCalibrated && !linearOpMode.isStopRequested) {
            linearOpMode.sleep(100)
        }
    }

    override val currentHeading: Heading
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return orientation.firstAngle.toDouble()
        }

    val currentPitch: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return orientation.thirdAngle.toDouble()
        }

    private val headingPIDController: PIDController by lazy {
        val controller = PIDController(linearOpMode, HEADING_PID_COEFFICIENTS)
        controller.setUpdateHandler { headingDifferenceFromTarget(targetHeading) }
        controller.start()
        controller
    }

    override fun headingCorrectedDrivePowers(baseDrivePowers: DrivePowers): DrivePowers {
        val drivePowers = DrivePowers()
        val headingCorrection = headingPIDController.output
        drivePowers.frontLeft = baseDrivePowers.frontLeft - headingCorrection
        drivePowers.frontRight = baseDrivePowers.frontRight + headingCorrection
        drivePowers.rearLeft = baseDrivePowers.rearLeft - headingCorrection
        drivePowers.rearRight = baseDrivePowers.rearRight + headingCorrection
        return drivePowers
    }

    companion object {
        const val MINIMUM_DRIVE_POWER = 0.15
        val HEADING_PID_COEFFICIENTS = PIDCoefficients(0.025, 0.00004, 0.15)
    }

}