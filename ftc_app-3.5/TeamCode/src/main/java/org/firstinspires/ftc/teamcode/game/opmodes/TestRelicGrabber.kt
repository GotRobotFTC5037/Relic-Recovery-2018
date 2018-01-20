package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.game.robots.TestBedBot

@TeleOp(name = "QuarterNote")
class TestRelicGrabber: LinearOpMode() {
    override fun runOpMode() {
        val robot = TestBedBot(this)
        waitForStart()
        while (opModeIsActive()) {
            robot.relicGrabber.setWinchPower(gamepad1.left_stick_y.toDouble())
            robot.relicGrabber.setCamPower(gamepad1.right_stick_y.toDouble())
            when {
                gamepad1.a -> robot.relicGrabber.setGrabberPosistion(1.0)
                gamepad1.b -> robot.relicGrabber.setGrabberPosistion(0.0)
            }
        }
    }

}