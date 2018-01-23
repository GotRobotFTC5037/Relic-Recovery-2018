package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.robots.Coda
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class CodaTeleOp : LinearOpMode() {

    private val robot: Coda by lazy { Coda(this) }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot.setup()
        waitForStart()
        robot.driveTrain.shouldCorrectHeading = false

        while (opModeIsActive()) {
            updateDriveDirection()
            updateGlyphGrabberState()
            updateLiftPower()
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

        linearPower = abs(Math.pow(linearPower, 3.0)) * Math.signum(linearPower)
        strafePower = abs(Math.pow(strafePower, 3.0)) * Math.signum(strafePower)
        turnPower = abs(Math.pow(turnPower, 3.0)) * Math.signum(turnPower)

        robot.driveTrain.setMovementPowers(linearPower, strafePower, turnPower)

        if (
            robot.glyphGrabber.currentState == GlyphGrabber.GlyphGrabberState.RELEASE &&
            abs(linearPower) > 0.75
        ) {
            robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.SMALL_OPEN)
        }
    }

    private fun updateGlyphGrabberState() {
        val gamepad = if (gamepad2IsRegistered()) gamepad2 else gamepad1

        when {
            gamepad.a -> robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED)
            gamepad.b -> robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.RELEASE)
        }
    }

    private fun updateLiftPower() {
        if (gamepad2IsRegistered()) {
            val liftPower = -gamepad2.left_stick_y.toDouble()
            robot.lift.setPower(liftPower)
        } else {
            when {
                gamepad1.dpad_up -> robot.lift.setPosition(CodaLift.LiftPosition.FIRST_LEVEL)
                gamepad1.dpad_down -> robot.lift.drop()
                else -> robot.lift.setPower(0.0)
            }
        }
    }

}

fun LinearOpMode.gamepad2IsRegistered() = gamepad2.id != Gamepad.ID_UNASSOCIATED
