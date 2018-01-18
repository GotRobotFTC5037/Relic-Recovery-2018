package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.components.RelicRecoveryRobot

@Autonomous
class Test: LinearOpMode() {

    override fun runOpMode() {

        // Setup the robot.
        val robot = RelicRecoveryRobot(this)
        waitForStart()

        while (opModeIsActive()) {

        }
    }

}