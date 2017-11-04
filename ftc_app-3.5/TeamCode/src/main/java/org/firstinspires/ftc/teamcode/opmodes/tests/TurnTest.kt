package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Disabled
@Autonomous(name = "Turn Test", group = "Tests")
class TurnTest : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.setup(this)
        waitForStart()
        robot.turn(0.45, 90)
        robot.turn(0.45, -90)
    }

}
