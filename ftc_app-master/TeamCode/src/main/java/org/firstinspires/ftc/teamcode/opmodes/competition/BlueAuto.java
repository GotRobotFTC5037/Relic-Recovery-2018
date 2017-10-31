package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot;

@Autonomous(name = "Blue Autonomous")
public class BlueAuto extends LinearOpMode {

    @Override public void runOpMode() throws InterruptedException {
        RelicRecoveryRobot robot = new RelicRecoveryRobot();
        robot.setup(this);

        waitForStart();

        // Detect the crypto box to put the glyph in.
        while(robot.getPictographIdentifier().identifyPictograph() == RelicRecoveryVuMark.UNKNOWN && opModeIsActive()) {
            idle();
        }

        RelicRecoveryVuMark pictograph = robot.getPictographIdentifier().identifyPictograph();

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
