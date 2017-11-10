package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.AutoTransitioner
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import org.firstinspires.ftc.teamcode.robots.Robot

@Autonomous(name = "Autonomous")
class RelicRecoveryAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        RelicRecoveryRobot.setup(this)
        val allianceColor = RelicRecoveryRobot.getAllianceColor()
        when (allianceColor) {
            Robot.AllianceColor.BLUE -> AutoTransitioner.transitionOnStop(this, "Blue Autonomous")
            Robot.AllianceColor.RED -> AutoTransitioner.transitionOnStop(this, "Red Autonomous")
            Robot.AllianceColor.UNKNOWN -> requestOpModeStop()
        }

        requestOpModeStop()
    }

}
