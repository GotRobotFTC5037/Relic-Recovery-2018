package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.game.robots.TestBedBot

@TeleOp(name = "Test: Relic Grabber")
@Disabled
class TestRelicGrabber: LinearOpMode() {
    override fun runOpMode() {
        val robot = TestBedBot(this)
        robot.setup()
        waitForStart()
        while (opModeIsActive()) {
            robot.relicGrabber.setWinchPower(gamepad1.left_stick_y.toDouble())
            robot.relicGrabber.setCamPower(gamepad1.right_stick_y.toDouble())
            when {
                gamepad1.a -> robot.relicGrabber.setGrabberPosition(1.0)
                gamepad1.b -> robot.relicGrabber.setGrabberPosition(0.0)
            }
        }
    }

}