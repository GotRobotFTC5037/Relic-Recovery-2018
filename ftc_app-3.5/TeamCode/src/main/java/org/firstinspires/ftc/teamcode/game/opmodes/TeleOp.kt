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
    private val holdDpadDown : ElapsedTime by lazy { ElapsedTime() }

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

        linearPower = abs(Math.pow(linearPower, 3.0)) * Math.signum(linearPower)
        strafePower = abs(Math.pow(strafePower, 3.0)) * Math.signum(strafePower)
        turnPower = abs(Math.pow(turnPower, 3.0)) * Math.signum(turnPower)

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
        val gamepad = if (gamepad2IsRegistered()) gamepad2 else gamepad1

        when {
            holdDpadDown.milliseconds() > 500 -> {
                robot.lift.setPosition(CodaLift.LiftPosition.BOTTOM)
                holdDpadDown.reset()
            }

            gamepad.dpad_down -> robot.lift.setPosition(CodaLift.LiftPosition.FIRST_LEVEL)
            gamepad.dpad_right -> {
                robot.lift.setPosition(CodaLift.LiftPosition.SECOND_LEVEL)
                holdDpadDown.reset()
            }

            gamepad.dpad_left -> {
                robot.lift.setPosition(CodaLift.LiftPosition.THIRD_LEVEL)
                holdDpadDown.reset()
            }

            gamepad.dpad_up -> {
                robot.lift.setPosition(CodaLift.LiftPosition.FORTH_LEVEL)
                holdDpadDown.reset()
            }
        }
    }

}

fun LinearOpMode.gamepad2IsRegistered(): Boolean {
    return gamepad2.id != Gamepad.ID_UNASSOCIATED
            && gamepad2.id != Gamepad.ID_SYNTHETIC
}

