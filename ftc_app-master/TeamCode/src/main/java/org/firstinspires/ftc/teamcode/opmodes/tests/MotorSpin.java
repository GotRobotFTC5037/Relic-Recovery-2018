package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot;

@Autonomous
public class MotorSpin extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RelicRecoveryRobot robot = new RelicRecoveryRobot();
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
