package org.firstinspires.ftc.teamcode.OpModes.Competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Robots.MecanumRobot;

@Autonomous(name = "Blue Autonomous")
public class BlueAuto extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        MecanumRobot robot = new MecanumRobot();
        robot.setup(this);

        waitForStart();

        // Detect the crypto box to put the glyph in.
        while(robot.identifier.identifyPictograph() == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {
            idle();
        }

        RelicRecoveryVuMark pictograph = robot.identifier.identifyPictograph();

        // Detect the color of the jewels

        // Knock off the jewel

        // Drive to the crypto boxes

        // Detect the column, drive to it and deliver glyph
        switch(pictograph) {
            case LEFT: break;
            case CENTER: break;
            case RIGHT: break;

            default: break; // This should never happen. If it does we need to display an error.
        }
    }

}
