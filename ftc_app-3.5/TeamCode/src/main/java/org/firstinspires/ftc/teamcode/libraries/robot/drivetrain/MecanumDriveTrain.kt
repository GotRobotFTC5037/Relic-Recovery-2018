package org.firstinspires.ftc.teamcode.libraries.robot.drivetrain

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.Position
import kotlin.concurrent.thread
import kotlin.math.abs

typealias Heading = Double
typealias MotorPower = Double

abstract class MecanumDriveTrain(override val linearOpMode: LinearOpMode) : DriveTrain {

    abstract val frontLeftMotor: DcMotor
    abstract val frontRightMotor: DcMotor
    abstract val rearLeftMotor: DcMotor
    abstract val rearRightMotor: DcMotor

    var targetHeading: Heading = 0.0
    var shouldCorrectHeading = true
    private var isActivelyTurning = false
    abstract val currentHeading: Heading

    protected val drivePowers: DrivePowers = DrivePowers()

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
    fun linearDrive(power: MotorPower) {
        drivePowers.frontLeft = power
        drivePowers.frontRight = power
        drivePowers.rearLeft = power
        drivePowers.rearRight = power
    }

    /**
     * Strafes the drive train by using the provided power.
     */
    fun strafeDrive(power: MotorPower) {
        drivePowers.frontLeft = -power
        drivePowers.frontRight = power
        drivePowers.rearLeft = power
        drivePowers.rearRight = -power
    }

    /**
     * Turns the drive train by using the provided power.
     */
    fun turn(power: MotorPower) {
        drivePowers.frontLeft = -power
        drivePowers.frontRight = power
        drivePowers.rearLeft = -power
        drivePowers.rearRight = power
    }

    /**
     * Linearly drives for the provided duration.
     */
    fun linearTimeDrive(duration: Long, power: MotorPower) {
        linearDrive(power)
        linearOpMode.sleep(duration)
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
        drivePowers.frontRight = adjustedLinearPower - adjustedStrafePower - adjustedTurnPower
        drivePowers.rearLeft = adjustedLinearPower - adjustedStrafePower + adjustedTurnPower
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

    /**
     * Moves the drive train to the specified position in cm.
     */
    fun driveToPosition(position: Position, power: MotorPower) {
        TODO("not implemented")
    }

    /**
     * Turns the drive train to the specified heading.
     */
    fun turnToHeading(targetHeading: Heading, power: MotorPower) {
        isActivelyTurning = true

        val targetHeadingDifference = headingDifferenceFromTarget(targetHeading)
        if (targetHeadingDifference > 0) {
            turn(abs(power))
            while (
                headingDifferenceFromTarget(targetHeading) > 0.0 &&
                !linearOpMode.isStopRequested
            ) {
                linearOpMode.idle()
            }
        } else if (targetHeadingDifference < 0) {
            turn(-abs(power))
            while (
                headingDifferenceFromTarget(targetHeading) < 0.0 &&
                !linearOpMode.isStopRequested
            ) {
                linearOpMode.idle()
            }
        }

        this.targetHeading = targetHeading
        isActivelyTurning = false
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