package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.lib.powercontroller.ProportionalPowerController
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.Heading
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.MecanumDriveTrain

class CodaDriveTrain(linearOpMode: LinearOpMode) : MecanumDriveTrain(linearOpMode) {

    override val frontLeftMotor: DcMotor by lazy {
        val motor = hardwareMap.dcMotor.get("front left motor")
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor
    }

    override val frontRightMotor: DcMotor by lazy {
        val motor = hardwareMap.dcMotor.get("front right motor")
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
         motor
    }

    override val rearLeftMotor: DcMotor by lazy {
        val motor = hardwareMap.dcMotor.get("rear left motor")
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor
    }

    override val rearRightMotor: DcMotor by lazy {
        val motor = hardwareMap.dcMotor.get("rear right motor")
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor
    }

    override val minimumDrivePower: Double
        get() = 0.2

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
            return -orientation.thirdAngle.toDouble()
        }

    private val headingController by lazy {
        val controller = ProportionalPowerController(HEADING_CORRECTION_GAIN)
        controller.errorValueHandler = {
            headingDifferenceFromTarget(targetHeading)
        }
        controller
    }

    private fun clipPowerAmplitude() {

    }

    override fun headingCorrectedDrivePowers(baseDrivePowers: DrivePowers): DrivePowers {
        val drivePowers = DrivePowers()
        val headingCorrection = headingController.outputPower
        drivePowers.frontLeft = baseDrivePowers.frontLeft - headingCorrection
        drivePowers.frontRight = baseDrivePowers.frontRight + headingCorrection
        drivePowers.rearLeft = baseDrivePowers.rearLeft - headingCorrection
        drivePowers.rearRight = baseDrivePowers.rearRight + headingCorrection
        return drivePowers
    }

    companion object {
        private const val HEADING_CORRECTION_GAIN = 0.03
    }

}