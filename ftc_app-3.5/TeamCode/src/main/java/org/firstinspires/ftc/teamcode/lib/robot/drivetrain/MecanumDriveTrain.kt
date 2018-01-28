package org.firstinspires.ftc.teamcode.lib.robot.drivetrain

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.lib.powercontroller.PowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.StaticPowerController
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.max

typealias Heading = Double
typealias MotorPower = Double

abstract class MecanumDriveTrain(override val linearOpMode: LinearOpMode) : DriveTrain {

    abstract val frontLeftMotor: DcMotor
    abstract val frontRightMotor: DcMotor
    abstract val rearLeftMotor: DcMotor
    abstract val rearRightMotor: DcMotor

    open var targetHeading: Heading = 0.0
    var shouldCorrectHeading = true
    private var isActivelyTurning = false
    abstract val currentHeading: Heading

    private val drivePowers: DrivePowers = DrivePowers()

    /**
     * Used to contain powers for the drive train's motors.
     */
    class DrivePowers {
        var frontLeft: MotorPower = 0.0
        var frontRight: MotorPower = 0.0
        var rearLeft: MotorPower = 0.0
        var rearRight: MotorPower = 0.0
    }

    /**
     * Linearly drives the drive train by using the provided power.
     */
    fun linearDriveAtPower(power: MotorPower) {
        drivePowers.frontLeft = power
        drivePowers.frontRight = power
        drivePowers.rearLeft = power
        drivePowers.rearRight = power
    }

    /**
     * Strafes the drive train by using the provided power.
     */
    fun strafeDriveAtPower(power: MotorPower) {
        drivePowers.frontLeft = -power
        drivePowers.frontRight = power
        drivePowers.rearLeft = power
        drivePowers.rearRight = -power
    }

    /**
     * Turns the drive train by using the provided power.
     */
    fun turnAtPower(power: MotorPower) {
        drivePowers.frontLeft = -power
        drivePowers.frontRight = power
        drivePowers.rearLeft = -power
        drivePowers.rearRight = power
    }

    /**
     * Linearly drives for the provided duration.
     */
    @Deprecated("Avoid using time based driving whenever possible.")
    fun linearTimeDrive(duration: Long, power: MotorPower) {
        linearDriveAtPower(power)
        linearOpMode.sleep(duration)
        stop()
    }

    /**
     * Moves the drive train by using the provided powers for each action.
     */
    fun setMovementPowers(
        linearPower: MotorPower,
        strafePower: MotorPower,
        turnPower: MotorPower
    ) {
        val max = Math.abs(linearPower) + Math.abs(strafePower) + Math.abs(turnPower)

        var adjustedLinearPower: MotorPower = linearPower
        var adjustedStrafePower: MotorPower = strafePower
        var adjustedTurnPower: MotorPower = turnPower

        if (max > 1.0) {
            adjustedLinearPower /= max
            adjustedStrafePower /= max
            adjustedTurnPower /= max
        }

        drivePowers.frontLeft = adjustedLinearPower + adjustedStrafePower - adjustedTurnPower
        drivePowers.frontRight = adjustedLinearPower - adjustedStrafePower + adjustedTurnPower
        drivePowers.rearLeft = adjustedLinearPower - adjustedStrafePower - adjustedTurnPower
        drivePowers.rearRight = adjustedLinearPower + adjustedStrafePower + adjustedTurnPower

        if (turnPower == 0.0) isActivelyTurning = false
    }

    /**
     * Stops all of the drive motors
     */
    override fun stop() {
        drivePowers.frontLeft = 0.0
        drivePowers.frontRight = 0.0
        drivePowers.rearLeft = 0.0
        drivePowers.rearRight = 0.0
    }

    fun resetEncoders() {
        frontLeftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rearLeftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rearRightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rearLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        rearRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun currentLinearEncoderPosition(): Int {
        val maxLeft = max(frontLeftMotor.currentPosition, rearLeftMotor.currentPosition)
        val maxRight = max(frontRightMotor.currentPosition, rearRightMotor.currentPosition)
        return max(maxLeft, maxRight)
    }

    /**
     * Drives linearly until the specified encoder value as be reached.
     */
    fun linearEncoderDrive(encoderValue: Int, power: MotorPower) {
        resetEncoders()
        if (encoderValue > 0) {
            linearDriveAtPower(abs(power))
            while (currentLinearEncoderPosition() < encoderValue && !linearOpMode.isStopRequested) {
                linearOpMode.idle()
            }
        } else if (encoderValue < 0) {
            linearDriveAtPower(-abs(power))
            while (currentLinearEncoderPosition() > encoderValue && !linearOpMode.isStopRequested) {
                linearOpMode.idle()
            }
        }
        stop()
    }

    /**
     * Turns the drive train to the specified heading.
     */
    fun turnToHeading(targetHeading: Heading, controller: PowerController) {

        // Prepare to turn
        isActivelyTurning = true
        controller.target = targetHeading
        controller.inputValueHandler = { currentHeading }

        // Determine which direction the robot will be turning in.
        val targetHeadingDifference = headingDifferenceFromTarget(targetHeading)
        if (targetHeadingDifference > 0) {
            while (
                headingDifferenceFromTarget(targetHeading) > 0.0 &&
                !linearOpMode.isStopRequested
            ) {
                turnAtPower(controller.output)
                linearOpMode.idle()
            }
        } else if (targetHeadingDifference < 0) {
            turnAtPower(controller.output)
            while (
                headingDifferenceFromTarget(targetHeading) < 0.0 &&
                !linearOpMode.isStopRequested
            ) {
                turnAtPower(controller.output)
                linearOpMode.idle()
            }
        }

        this.targetHeading = targetHeading
        isActivelyTurning = false
    }

    fun turnToHeading(targetHeading: Heading, power: MotorPower) {
        if (headingDifferenceFromTarget(targetHeading) > 0.0) {
            turnToHeading(targetHeading, StaticPowerController(abs(power)))
        } else if (headingDifferenceFromTarget(targetHeading) < 0.0) {
            turnToHeading(targetHeading, StaticPowerController(-abs(power)))
        }
    }

    /**
     * Starts a thread that continually updates the powers of the drive motors.
     */
    fun startUpdatingDrivePowers() {
        thread(start = true) {
            while (linearOpMode.opModeIsActive()) {
                val drivePowers = if (shouldCorrectHeading && !isActivelyTurning) {
                    headingCorrectedDrivePowers(this.drivePowers)
                } else {
                    this.drivePowers
                }

                frontLeftMotor.power = drivePowers.frontLeft
                frontRightMotor.power = drivePowers.frontRight
                rearLeftMotor.power = drivePowers.rearLeft
                rearRightMotor.power = drivePowers.rearRight
            }
        }
    }

    /**
     * Returns the shortest heading difference from a specified angle.
     */
    protected fun headingDifferenceFromTarget(target: Double): Double {
        val currentHeading = currentHeading
        val headingDifference = target - currentHeading
        return when {
            Math.abs(headingDifference) <= 180 -> headingDifference
            target > currentHeading -> target - (currentHeading + 360)
            target < currentHeading -> target - (currentHeading - 360)
            else -> 0.0 // This shouldn't ever happen.
        }
    }

    /**
     * Returns the drive powers after correction.
     */
    abstract fun headingCorrectedDrivePowers(baseDrivePowers: DrivePowers): DrivePowers

}