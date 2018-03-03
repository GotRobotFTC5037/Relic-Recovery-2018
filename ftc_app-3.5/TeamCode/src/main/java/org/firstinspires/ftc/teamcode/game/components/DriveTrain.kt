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
import kotlin.math.abs

class CodaDriveTrain(linearOpMode: LinearOpMode) : MecanumDriveTrain(linearOpMode) {

    override val frontLeftMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get("front left motor").apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override val frontRightMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get("front right motor").apply {
            direction = DcMotorSimple.Direction.FORWARD
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override val rearLeftMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get("rear left motor").apply {
            direction = DcMotorSimple.Direction.REVERSE
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    override val rearRightMotor: DcMotor by lazy {
        hardwareMap.dcMotor.get("rear right motor").apply {
            direction = DcMotorSimple.Direction.FORWARD
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    private val imu: BNO055IMU by lazy {
        hardwareMap.get(BNO055IMU::class.java, "imu").also {
            it.initialize(BNO055IMU.Parameters().apply { angleUnit = BNO055IMU.AngleUnit.DEGREES })
        }
    }

    override val minimumDrivePower = 0.25

    fun waitForGyroCalibration() {
        while (!imu.isGyroCalibrated && !linearOpMode.isStopRequested) {
            linearOpMode.sleep(100)
        }
    }

    override val currentHeading: Heading
        get() = imu.getAngularOrientation(
            AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle.toDouble()

    val currentPitch: Double
        get() = -imu.getAngularOrientation(
            AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).thirdAngle.toDouble()

    private val headingController by lazy {
        ProportionalPowerController(HEADING_CORRECTION_GAIN).apply {
            errorValueHandler = {
                headingDifferenceFromTarget(targetHeading)
            }
        }
    }

    override fun headingCorrectedDrivePowers(baseDrivePowers: DrivePowers): DrivePowers {
        val headingCorrection =
            if (abs(headingDifferenceFromTarget(targetHeading)) >= HEADING_CORRECTION_THRESHOLD) {
                headingController.outputPower
            } else {
                0.0
            }

        return DrivePowers().apply {
            frontLeft = baseDrivePowers.frontLeft - headingCorrection
            frontRight = baseDrivePowers.frontRight + headingCorrection
            rearLeft = baseDrivePowers.rearLeft - headingCorrection
            rearRight = baseDrivePowers.rearRight + headingCorrection
        }
    }

    companion object {
        private const val HEADING_CORRECTION_GAIN = 0.025
        private const val HEADING_CORRECTION_THRESHOLD = 2.0
    }

}