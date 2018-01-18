package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.libraries.components.RelicRecoveryRobot
import org.firstinspires.ftc.teamcode.libraries.components.attachments.GlyphGrabber
import java.lang.Math.pow
import java.lang.Math.signum
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class RelicRecoveryTeleOp : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "TeleOp"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val robot = RelicRecoveryRobot(this)
        robot.setup()
        waitForStart()
        robot.mecanumDriveTrain.startUpdatingDriveMotorPowers()
        robot.lift.beginHoldingPosition()

        while (opModeIsActive()) {

            // Gamepad 1: Movement
            /*
            when {

                gamepad1.dpad_up -> {
                    robot.setDrivePower(0.35); if (!robot.shouldCorrectHeading) {
                        robot.targetHeading = robot.heading; robot.shouldCorrectHeading = false
                    }
                }

                gamepad1.dpad_down -> {
                    robot.setDrivePower(-0.35); if (!robot.shouldCorrectHeading) {
                        robot.targetHeading = robot.heading; robot.shouldCorrectHeading = false
                    }
                }

                gamepad1.dpad_left -> {
                    robot.setStrafePower(-0.40); if (!robot.shouldCorrectHeading) {
                        robot.targetHeading = robot.heading; robot.shouldCorrectHeading = false
                    }
                }

                gamepad1.dpad_right -> {
                    robot.setStrafePower(0.40); if (!robot.shouldCorrectHeading) {
                        robot.targetHeading = robot.heading; robot.shouldCorrectHeading = false
                    }
                }


                else -> {
                    robot.shouldCorrectHeading = false

                    val x = gamepad1.left_stick_x.toDouble() * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.85 else 1.0
                    val y = gamepad1.left_stick_y.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.85 else 1.0
                    val z = gamepad1.right_stick_x.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.65 else 0.85

                    val xPower = abs(pow(x, 3.0)) * signum(x)
                    val yPower = abs(pow(y, 3.0)) * signum(y)
                    val zPower = abs(pow(z, 3.0)) * signum(z)

                    robot.setDirection(xPower, yPower, zPower)
                }
            }
            */

            val x = gamepad1.left_stick_x.toDouble() * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.85 else 1.0
            val y = gamepad1.left_stick_y.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.85 else 1.0
            val z = gamepad1.right_stick_x.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 0.65 else 0.85

            val xPower = abs(pow(x, 3.0)) * signum(x)
            val yPower = abs(pow(y, 3.0)) * signum(y)
            val zPower = abs(pow(z, 3.0)) * signum(z)

            robot.mecanumDriveTrain.setDirection(xPower, yPower, zPower)

            // Gamepad 2: Attachments
            //robot.lift.manuallyMove((gamepad2.left_stick_y * -1).toDouble())

            //when {
                //gamepad2.dpad_down -> robot.lift.drop()
            //}

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
