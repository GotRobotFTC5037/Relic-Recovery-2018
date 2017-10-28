package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.MecanumRobot;

@Autonomous
public class MotorSpin extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.setup(this);

        waitForStart();

        robot.setWinchPower(0.5);

        while(opModeIsActive()){
            telemetry.addData("Encoder", robot.mWinchMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
