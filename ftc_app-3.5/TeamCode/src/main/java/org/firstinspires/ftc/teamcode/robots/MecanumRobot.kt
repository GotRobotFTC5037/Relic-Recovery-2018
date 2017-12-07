package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import java.lang.Math.abs
import kotlin.concurrent.thread

open class MecanumRobot : Robot() {

    companion object {
        val MINIMUM_DRIVE_POWER = 0.10
        val HEADING_CORRECTION_COEFFICIENT = 0.05
    }

    private lateinit var frontLeftMotor: DcMotor
    private lateinit var frontRightMotor: DcMotor
    private lateinit var backLeftMotor: DcMotor
    private lateinit var backRightMotor: DcMotor
    private lateinit var imu: BNO055IMU

    private var frontLeftMotorPower = 0.0
    private var frontRightMotorPower = 0.0
    private var backLeftMotorPower = 0.0
    private var backRightMotorPower = 0.0

    var shouldCorrectHeading = true
    var targetHeading = 0.0

    val heading: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return orientation.firstAngle.toDouble()
        }

    val roll: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return orientation.secondAngle.toDouble()
        }

    val pitch: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return orientation.thirdAngle.toDouble()
        }

    override fun setup(hardwareMap: HardwareMap) {
        shouldCorrectHeading = true
        targetHeading = 0.0

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

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        imu.initialize(imuParameters)
    }

    private fun headingDifferenceFromTarget(target: Double): Double {
        val currentHeading = heading
        val headingDifference = target - currentHeading
        return when {
            abs(headingDifference) <= 180 -> headingDifference
            target > currentHeading -> target - (currentHeading + 360)
            target < currentHeading -> target - (currentHeading - 360)
            else -> 0.0 // This shouldn't ever happen.
        }
    }

    fun startUpdatingDriveMotorPowers() {
        thread(start = true) {
            while(!linearOpMode.isStopRequested) {
                val headingCorrection = if(shouldCorrectHeading) headingCorrectionPower() else 0.0

                if(Math.abs(frontLeftMotorPower) < MINIMUM_DRIVE_POWER && frontLeftMotorPower != 0.0)
                    frontLeftMotor.power = MINIMUM_DRIVE_POWER * Math.signum(frontLeftMotorPower)
                else
                    frontLeftMotor.power = Range.clip(frontLeftMotorPower - headingCorrection, -1.0, 1.0)

                if(Math.abs(frontRightMotorPower) < MINIMUM_DRIVE_POWER && frontRightMotorPower != 0.0)
                    frontRightMotor.power = MINIMUM_DRIVE_POWER * Math.signum(frontRightMotorPower)
                else
                    frontRightMotor.power = Range.clip(frontRightMotorPower + headingCorrection, -1.0, 1.0)

                if(Math.abs(backLeftMotorPower) < MINIMUM_DRIVE_POWER && backLeftMotorPower != 0.0)
                    backLeftMotor.power = MINIMUM_DRIVE_POWER * Math.signum(backLeftMotorPower)
                else
                    backLeftMotor.power = Range.clip(backLeftMotorPower - headingCorrection, -1.0, 1.0)

                if(Math.abs(backRightMotorPower) < MINIMUM_DRIVE_POWER && backRightMotorPower != 0.0)
                    backRightMotor.power = MINIMUM_DRIVE_POWER * Math.signum(backRightMotorPower)
                else
                    backRightMotor.power = Range.clip(backRightMotorPower + headingCorrection, -1.0, 1.0)

                linearOpMode.sleep(10)
            }
        }
    }

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

    fun setDirection(x: Double, y: Double, z: Double = 0.0) {
        var adjustedX = x
        var adjustedY = y
        var adjustedZ = z

        val max = Math.abs(adjustedY) + Math.abs(adjustedX) + Math.abs(adjustedZ)
        if(max > 1.0) {
            adjustedX /= max
            adjustedY /= max
            adjustedZ /= max
        }

        frontLeftMotorPower =  adjustedY + adjustedX - adjustedZ
        backLeftMotorPower = adjustedY - adjustedX - adjustedZ
        frontRightMotorPower = adjustedY - adjustedX + adjustedZ
        backRightMotorPower = adjustedY + adjustedX + adjustedZ
    }

    override fun turn(power: Double, degrees: Double) {
        if(!linearOpMode.isStopRequested) {

            shouldCorrectHeading = false
            targetHeading = degrees

            val targetHeadingDifference = headingDifferenceFromTarget(degrees)

            when {
                targetHeadingDifference > 0.0 -> {
                    setTurnPower(abs(power))
                    while (headingDifferenceFromTarget(targetHeading) > 0.0 && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                targetHeadingDifference < 0.0 -> {
                    setTurnPower(-abs(power))
                    while (headingDifferenceFromTarget(targetHeading) < 0.0 && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }
            }

            shouldCorrectHeading = true
        }

        stopAllDriveMotors()
    }

    open fun waitForGyroCalibration() {
        while (!imu.isGyroCalibrated && linearOpMode.opModeIsActive()) {
            linearOpMode.sleep(10)
        }
    }

    private fun headingCorrectionPower(): Double = headingDifferenceFromTarget(targetHeading) * HEADING_CORRECTION_COEFFICIENT

}

