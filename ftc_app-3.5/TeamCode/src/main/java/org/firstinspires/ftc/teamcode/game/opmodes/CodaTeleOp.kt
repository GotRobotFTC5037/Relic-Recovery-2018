package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabbers
import org.firstinspires.ftc.teamcode.game.robots.Coda
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class CodaTeleOp : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "TeleOp"
    }

    lateinit var robot: Coda

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        robot = Coda(this)
        robot.setup()
        waitForStart()

        while (opModeIsActive()) {

            // Gamepad 1: Movement
            updateDriveDirection()
            updateTurningState()


            // Gamepad 2: Attachments
            updateLiftPower()
            updateGlyphGrabberState()
        }

        robot.driveTrain.stop()
    }

    private fun updateDriveDirection() {
        var xPower = gamepad1.left_stick_x.toDouble()
        var yPower = gamepad1.left_stick_y.toDouble()
        var zPower = gamepad1.right_stick_x.toDouble()

        if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) {
            xPower *= 0.85
            yPower *= 0.85
            zPower *= 0.65
        } else {
            xPower *= 1.00
            yPower *= 1.00
            zPower *= 0.85
        }

        xPower = abs(Math.pow(xPower, 3.0)) * Math.signum(xPower)
        yPower = abs(Math.pow(yPower, 3.0)) * Math.signum(yPower)
        zPower = abs(Math.pow(zPower, 3.0)) * Math.signum(zPower)

        if (zPower == 0.0) {
            if (!robot.driveTrain.shouldCorrectHeading) {
                robot.driveTrain.shouldCorrectHeading = true
                robot.driveTrain.targetHeading = robot.driveTrain.heading
            }
        } else {
            robot.driveTrain.shouldCorrectHeading = false
        }

        robot.driveTrain.setDirection(xPower, yPower, zPower)
    }

    private fun updateLiftPower() {
        val liftPower = (gamepad2.left_stick_y * -1).toDouble()
        robot.lift.manuallyMove(liftPower)
    }

    private fun updateGlyphGrabberState() {
        when {
            gamepad2.a -> robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.CLOSED)
            gamepad2.y -> robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.RELEASE)
            gamepad2.b -> robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.OPEN)
            gamepad2.x -> robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.SMALL_OPEN)
        }
    }

    private fun updateTurningState() {
        when {
            gamepad1.x -> robot.driveTrain.turnTo(0.0, 1.00)
            gamepad1.a -> robot.driveTrain.turnTo(90.0, 1.00)
            gamepad1.b -> robot.driveTrain.turnTo(180.0, 1.00)
            gamepad1.y -> robot.driveTrain.turnTo(-90.0, 1.00)
        }
    }

}
