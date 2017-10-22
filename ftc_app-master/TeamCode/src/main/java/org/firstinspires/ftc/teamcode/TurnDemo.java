package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Turn Demo")
public class TurnDemo extends LinearOpMode {

    private TestRobot robot = new TestRobot();

    @Override public void runOpMode() throws InterruptedException {
        robot.setup(hardwareMap);
        waitForStart();
        robot.turn(0.45, Math.PI / 2);
    }
}
