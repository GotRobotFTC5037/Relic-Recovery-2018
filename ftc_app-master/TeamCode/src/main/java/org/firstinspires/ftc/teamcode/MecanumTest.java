package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Mecanum Test", group="Tests")
public class MecanumTest extends LinearOpMode {

    private MecanumRobot robot = new MecanumRobot();

    @Override public void runOpMode() throws InterruptedException {
        robot.setup(hardwareMap);
        waitForStart();

        while(opModeIsActive()) {

            double xPos = gamepad1.right_stick_x;
            double yPos = gamepad1.right_stick_y * -1;

            if ((xPos != 0 || yPos != 0) && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                robot.setDirection(xPos, yPos);
            } else if (gamepad1.right_bumper) {
                robot.setTurnPower(-1);
            } else if (gamepad1.left_bumper) {
                robot.setTurnPower(1);
            } else {
                robot.setDirection(0, 0);
            }

            idle();
        }

    }
}
