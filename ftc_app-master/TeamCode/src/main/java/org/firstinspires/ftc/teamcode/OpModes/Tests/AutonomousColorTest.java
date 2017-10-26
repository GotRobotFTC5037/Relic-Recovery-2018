package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Robots.TestRobot;

@Disabled
@Autonomous(name = "TB - Color Test", group = "Tests")
public class AutonomousColorTest extends LinearOpMode {

    private TestRobot robot = new TestRobot();
    private final static double POWER = 0.20;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.setup(this);
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "color distance");

        waitForStart();

        while (opModeIsActive()) {
            double redValue = colorSensor.red();
            double blueValue = colorSensor.blue();

            if (blueValue > redValue) {
                robot.setDrivePower(POWER);
            } else if (blueValue < redValue) {
                robot.setDrivePower(-POWER);
            } else {
                robot.setDrivePower(0);
            }
        }
    }

}