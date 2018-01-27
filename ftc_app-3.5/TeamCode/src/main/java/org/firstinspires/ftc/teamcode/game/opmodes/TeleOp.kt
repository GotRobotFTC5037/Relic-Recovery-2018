package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.robots.Coda
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class CodaTeleOp : LinearOpMode() {

    private val robot: Coda by lazy { Coda(this) }

    private val lastManualHeadingUpdate: ElapsedTime by lazy { ElapsedTime() }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot.setup()
        waitForStart()
        robot.lift.startSettingMotorPowers()

        while (opModeIsActive()) {
            updateDriveDirection()
            updateGlyphGrabberState()
            updateLift()
        }

        robot.driveTrain.stop()
    }

    private fun updateDriveDirection() {
        var linearPower = -gamepad1.left_stick_y.toDouble()
        var strafePower = gamepad1.left_stick_x.toDouble()
        var turnPower = -gamepad1.right_stick_x.toDouble()

        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) {
            linearPower *= 0.85
            strafePower *= 0.85
            turnPower *= 0.65
        } else {
            linearPower *= 1.00
            strafePower *= 1.00
            turnPower *= 0.85
        }

        linearPower = abs(Math.pow(linearPower, 2.0)) * Math.signum(linearPower)
        strafePower = abs(Math.pow(strafePower, 2.0)) * Math.signum(strafePower)
        turnPower = abs(Math.pow(turnPower, 2.0)) * Math.signum(turnPower)

        robot.driveTrain.setMovementPowers(linearPower, strafePower, turnPower)

        if (
            robot.glyphGrabber.currentState == GlyphGrabber.GlyphGrabberState.RELEASE &&
            linearPower > 0.65
        ) {
            robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.SMALL_OPEN)
        }

        if (turnPower != 0.0) lastManualHeadingUpdate.reset()

        if (lastManualHeadingUpdate.milliseconds() >= 750) {
            robot.driveTrain.targetHeading = robot.driveTrain.currentHeading
            robot.driveTrain.shouldCorrectHeading = true
        } else {
            robot.driveTrain.shouldCorrectHeading = false
        }
    }

    private fun updateGlyphGrabberState() {
        val gamepad = if (gamepad2IsRegistered()) gamepad2 else gamepad1

        when {
            gamepad.a -> robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED)
            gamepad.b -> robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.RELEASE)
            gamepad.x -> robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.SMALL_OPEN)
        }
    }

    private fun updateLift() {
        val gamepad = if (gamepad2IsRegistered()) {
            val liftPower = -gamepad2.left_stick_y.toDouble()
            if (liftPower != 0.0) {
                robot.lift.shouldHoldLiftPosition = false
                robot.lift.setPower(liftPower)
                val position = CodaLift.LiftPosition.MANUAL
                position.value = robot.lift.motor.currentPosition
                robot.lift.targetPosition = position
            } else {
                robot.lift.shouldHoldLiftPosition = true
            }

            gamepad2
        } else {
            robot.lift.shouldHoldLiftPosition = true
            gamepad1
        }

        when {
            gamepad.dpad_up -> {
                while (gamepad.dpad_up) {
                    idle()
                }
                robot.lift.elevate()
            }

            gamepad.dpad_down -> {
                while (gamepad.dpad_down) {
                    idle()
                }
                robot.lift.lower()
            }
        }
    }

}

fun LinearOpMode.gamepad2IsRegistered(): Boolean {
    return gamepad2.id != Gamepad.ID_UNASSOCIATED
            && gamepad2.id != Gamepad.ID_SYNTHETIC
}

