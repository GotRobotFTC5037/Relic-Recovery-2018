package org.firstinspires.ftc.teamcode.libraries.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

class RobotBuilder(val linearOpMode: LinearOpMode) {

    fun buildRobot(init: Robot.() -> Unit): Robot {
        val robot = Robot(linearOpMode)
        robot.init()
        return robot
    }

}