package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.TestRobot;

@Autonomous(name = "TB - Turn Test", group = "Tests")
public class TurnTest extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        TestRobot robot = new TestRobot();
        robot.setup(this);
        waitForStart();
        robot.turn(0.45, 90);
    }

}
