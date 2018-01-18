package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.robots.Coda
import java.lang.Math.pow
import java.lang.Math.signum
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class CodaTeleOp : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "TeleOp"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val robot = Coda(this)
        robot.setup()
        waitForStart()

        while (opModeIsActive()) {

            // Gamepad 1: Movement
            val x = gamepad1.left_stick_x.toDouble() * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.85 else 1.0
            val y = gamepad1.left_stick_y.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.85 else 1.0
            val z = gamepad1.right_stick_x.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.65 else 0.85

            val xPower = abs(pow(x, 3.0)) * signum(x)
            val yPower = abs(pow(y, 3.0)) * signum(y)
            val zPower = abs(pow(z, 3.0)) * signum(z)

            robot.mecanumDriveTrain.setDirection(xPower, yPower, zPower)

            when {
                gamepad1.x -> robot.mecanumDriveTrain.targetHeading = 90.0
                gamepad1.a -> robot.mecanumDriveTrain.targetHeading = 0.0
                gamepad1.b -> robot.mecanumDriveTrain.targetHeading = -90.0
            }

            // Gamepad 2: Attachments
            val liftPower = (gamepad2.left_stick_y * -1).toDouble()
            robot.lift.manuallyMove(liftPower)

            when {
                gamepad2.a -> robot.glyphGrabber.setGlyphGrabberPosition(GlyphGrabber.GlyphGrabberPosition.CLOSED)
                gamepad2.y -> robot.glyphGrabber.setGlyphGrabberPosition(GlyphGrabber.GlyphGrabberPosition.RELEASE)
                gamepad2.b -> robot.glyphGrabber.setGlyphGrabberPosition(GlyphGrabber.GlyphGrabberPosition.OPEN)
                gamepad2.x -> robot.glyphGrabber.setGlyphGrabberPosition(GlyphGrabber.GlyphGrabberPosition.SMALL_OPEN)
            }
        }

        robot.mecanumDriveTrain.stopAllDriveMotors()
    }

}
