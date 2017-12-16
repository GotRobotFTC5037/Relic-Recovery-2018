package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
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
        val robot = RelicRecoveryRobot()
        robot.linearOpMode = this
        robot.setup(hardwareMap)
        robot.shouldCorrectHeading = false
        robot.printTeleOpInstructions()
        robot.waitForGyroCalibration()
        robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.READY)
        waitForStart()
        robot.start()

        robot.dropLift()
        robot.raiseJewelStick()

        while (opModeIsActive()) {
            // Gamepad 1: Movement
            when {
                gamepad1.dpad_up -> { robot.setDrivePower(0.20); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }
                gamepad1.dpad_down -> {robot.setDrivePower(-0.20); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }
                gamepad1.dpad_left -> { robot.setStrafePower(-0.65); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }
                gamepad1.dpad_right -> { robot.setStrafePower(0.65); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }

                else -> {
                    robot.shouldCorrectHeading = false
                    val x = gamepad1.left_stick_x.toDouble()
                    val y = gamepad1.left_stick_y.toDouble() * -1.0
                    val z = gamepad1.right_stick_x.toDouble() * -1.0

                    val xPower = abs(pow(x, 3.0)) * signum(x)
                    val yPower = abs(pow(y, 3.0)) * signum(y)
                    val zPower = abs(pow(z, 3.0)) * signum(z)

                    robot.setDirection(xPower, yPower, zPower)
                }
            }

            // Gamepad 2: Attachments
            val winchPower = (gamepad2.left_stick_y * -1).toDouble()
            robot.setLiftWinchPower(winchPower)

            when {
                gamepad2.a -> { robot.closeGlyphGrabbers(); robot.retractGlyphDeployer() }
                gamepad2.y -> { robot.releaseGlyphGrabbers(); robot.extendGlyphDeployer() }
                gamepad2.b -> { robot.openGlyphGrabbers(); robot.retractGlyphDeployer() }
                gamepad2.x -> { robot.smallOpenGlyphGrabbers(); robot.retractGlyphDeployer() }
            }
        }

        robot.stopAllDriveMotors()
    }

}
