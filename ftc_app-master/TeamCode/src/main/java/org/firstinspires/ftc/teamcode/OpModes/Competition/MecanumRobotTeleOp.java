package org.firstinspires.ftc.teamcode.OpModes.Competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.MecanumRobot;

@TeleOp public class MecanumRobotTeleOp extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.setup(this);

        waitForStart();

        while(opModeIsActive()) {
            double xPower = gamepad1.right_stick_x;
            double yPower = gamepad1.right_stick_y * -1;
            double zPower;

            if(gamepad1.left_bumper && !gamepad1.right_bumper) {
                zPower = -(1 - (Math.sqrt(2) / 2));
            } else if(!gamepad1.left_bumper && gamepad1.right_bumper) {
                zPower = (1 - (Math.sqrt(2) / 2));
            } else {
                zPower = 0;
            }

            robot.setDirection(xPower, yPower, zPower);

            idle();
        }

        robot.stop();
    }

}
