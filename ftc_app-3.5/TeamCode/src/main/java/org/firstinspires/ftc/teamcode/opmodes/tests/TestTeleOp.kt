package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robots.MecanumRobot

@TeleOp
class TestTeleOp: LinearOpMode() {
    override fun runOpMode() {
        val robot = MecanumRobot()
        robot.setup(this)
        waitForStart()
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

            robot.setDirection(xPower, yPower, zPower)
        }
    }
}