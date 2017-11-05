package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import org.firstinspires.ftc.teamcode.robots.Robot

@Autonomous(name = "Autonomous")
class RelicRecoveryAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.setup(this)
        robot.waitForGyroCalibration()

        val allianceColor = robot.getAllianceColor()
        when (allianceColor) {
            Robot.AllianceColor.BLUE -> { BlueAutonomous.setup(robot); BlueAutonomous.runOpMode() }
            Robot.AllianceColor.RED -> waitForStart()
            Robot.AllianceColor.UNKNOWN -> waitForStart()
        }

        requestOpModeStop()

    }

}
