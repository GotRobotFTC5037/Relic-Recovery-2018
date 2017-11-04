package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@TeleOp(name = "TeleOp")
class RelicRecoveryTeleOp : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.setup(this)
        robot.waitForGyroCalibration()

        waitForStart()

        robot.raiseJewelStick()

        while (opModeIsActive()) {
            val xPower = gamepad1.right_stick_x.toDouble()
            val yPower = (gamepad1.right_stick_y * -1).toDouble()
            val zPower: Double = if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                -Math.sqrt(2.0) / 2
            } else if (!gamepad1.left_bumper && gamepad1.right_bumper) {
                Math.sqrt(2.0) / 2
            } else {
                0.0
            }

            if (gamepad1.a) {
                robot.closeGlyphGrabbers()
            } else if (gamepad1.b) {
                robot.openGlyphGrabbers()
            }

            robot.setDirection(xPower, yPower, zPower)
            robot.setWinchPower((gamepad1.left_stick_y * -1).toDouble())
            idle()
        }
        robot.stop()
    }
}
