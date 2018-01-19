package org.firstinspires.ftc.teamcode.libraries.robot.drivetrains

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import kotlin.concurrent.thread
import kotlin.properties.Delegates

class MecanumDriveTrain(linearOpMode: LinearOpMode) : RobotDriveTrain(linearOpMode) {

    companion object {
        const val MINIMUM_DRIVE_POWER = 0.15
        const val HEADING_PROPORTIONAL_GAIN = 0.025
        const val HEADING_INTEGRAL_GAIN     = 0.04
        const val HEADING_DERIVATIVE_GAIN   = 0.00015
    }

    // Motors & Gyro
    private var frontLeftMotor: DcMotor
    private var frontRightMotor: DcMotor
    private var backLeftMotor: DcMotor
    private var backRightMotor: DcMotor
    private var imu: BNO055IMU

    // Motor Powers
    private var frontLeftMotorPower = 0.0
    private var frontRightMotorPower = 0.0
    private var backLeftMotorPower = 0.0
    private var backRightMotorPower = 0.0

    // Heading Correction, Turning & PID
    var targetHeading = 0.0
    private var previousTargetHeadingError = 0.0
    private var headingIntegral = 0.0
    private var timeSinceLastHeadingCorrection = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    var shouldCorrectHeading: Boolean by Delegates.observable(true) { _, _, _ ->
        timeSinceLastHeadingCorrection.reset()
    }

    private val heading: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return orientation.firstAngle.toDouble()
        }

    private val roll: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return -orientation.secondAngle.toDouble()
        }

    val pitch: Double
        get() {
            val orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
            return -orientation.thirdAngle.toDouble()
        }

    /**
     * Sets up the hardware needed in order to use the robot.
     */
    init {
        val hardwareMap = linearOpMode.hardwareMap

        shouldCorrectHeading = true
        targetHeading = 0.0

        frontLeftMotor = hardwareMap.dcMotor.get("front left motor")
        frontLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        frontLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        frontRightMotor = hardwareMap.dcMotor.get("front right motor")
        frontRightMotor.direction = DcMotorSimple.Direction.FORWARD
        frontRightMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backLeftMotor = hardwareMap.dcMotor.get("back left motor")
        backLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        backLeftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        backRightMotor = hardwareMap.dcMotor.get("back right motor")
        backRightMotor.direction = DcMotorSimple.Direction.FORWARD
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

    /**
     * Waits until the gyro is calibrated.
     */
    fun waitForGyroCalibration() {
        while (!imu.isGyroCalibrated && linearOpMode.opModeIsActive()) {
            linearOpMode.sleep(10)
        }
    }

    /**
     * Begins the thread that updates the power of the motors.
     */
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


    /**
     * Sets the power that the robot should drive forward or backwards.
     * @param power The power that the robot should drive at.
     */
    override fun setDrivePower(power: Double) {
        frontLeftMotorPower = power
        frontRightMotorPower = power
        backLeftMotorPower = power
        backRightMotorPower = power
    }

    /**
     * Sets the power that the robot should turn.
     * @param power The power that the robot should turn at.
     */
    override fun setTurnPower(power: Double) {
        frontLeftMotorPower = -power
        frontRightMotorPower = power
        backLeftMotorPower = -power
        backRightMotorPower = power
    }

    /**
     * Sets the power that the robot should drive left or right.
     * @param power The power that the robot should strafe at.
     */
    fun setStrafePower(power: Double) {
        frontLeftMotorPower = power
        frontRightMotorPower = -power
        backLeftMotorPower = -power
        backRightMotorPower = power
    }

    /**
     * Sets the power of the drive motors to 0. This does not stop the robot from correcting with
     * gyro correction. Set shouldCorrectHeading to false to stop gyro correction.
     */
    override fun stop() {
        frontLeftMotorPower = 0.0
        frontRightMotorPower = 0.0
        backLeftMotorPower = 0.0
        backRightMotorPower = 0.0
    }

    /**
     * Moves the robot in the direction of the specified vector.
     * @param x The x value of the power vector.
     * @param y The y value of the power vector.
     * @param z The turn power of the robot.
     */
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

    /**
     * Turns the robot to the specified angle.
     * @param power The power that the robot should turn at.
     * @param degrees the degrees the robot should move at.
     */
    fun turnTo(power: Double, degrees: Double) {

        if(!linearOpMode.isStopRequested) {
            linearOpMode.telemetry.log().add("Turning the robot to $degrees degrees")

            shouldCorrectHeading = false
            targetHeading = degrees

            val targetHeadingDifference = headingDifferenceFromTarget(degrees)

            when {
                targetHeadingDifference > 0.0 -> {
                    setTurnPower(Math.abs(power))
                    while (headingDifferenceFromTarget(targetHeading) > 0.0 && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }

                targetHeadingDifference < 0.0 -> {
                    setTurnPower(-Math.abs(power))
                    while (headingDifferenceFromTarget(targetHeading) < 0.0 && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                }
            }

            shouldCorrectHeading = true
        }

        stop()
    }

    /**
     * Calculates the shortest heading difference from a specified angle.
     * @param target The target angle in degrees.
     * @return The angle in degrees to turn in order to get to the specified angle.
     */
    private fun headingDifferenceFromTarget(target: Double): Double {
        val currentHeading = heading
        val headingDifference = target - currentHeading
        return when {
            Math.abs(headingDifference) <= 180 -> headingDifference
            target > currentHeading -> target - (currentHeading + 360)
            target < currentHeading -> target - (currentHeading - 360)
            else -> 0.0 // This shouldn't ever happen.
        }
    }

    /**
     * Calculates the power the robot should turn at in order to correct for the heading drift of
     * the robot.
     */
    private fun headingCorrectionPower(): Double {
        val dt = timeSinceLastHeadingCorrection.milliseconds() / 1000

        val error = headingDifferenceFromTarget(targetHeading)
        headingIntegral += error * dt
        val derivative = (previousTargetHeadingError - error) / dt

        val proportionalOutput = error * HEADING_PROPORTIONAL_GAIN
        val integralOutput = headingIntegral * HEADING_INTEGRAL_GAIN
        val derivativeOutput = derivative * HEADING_DERIVATIVE_GAIN
        val output = proportionalOutput + integralOutput + derivativeOutput

        linearOpMode.telemetry.addLine("Proportional: $proportionalOutput")
        linearOpMode.telemetry.addLine("Integral: $integralOutput")
        linearOpMode.telemetry.addLine("Derivative: $derivativeOutput")
        linearOpMode.telemetry.addLine()
        linearOpMode.telemetry.addLine("Output: $output")
        linearOpMode.telemetry.update()

        timeSinceLastHeadingCorrection.reset()

        return output
    }
}