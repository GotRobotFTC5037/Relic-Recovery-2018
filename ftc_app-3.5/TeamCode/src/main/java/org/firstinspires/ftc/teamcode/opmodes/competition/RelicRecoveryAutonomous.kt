package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.AutoTransitioner
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import org.firstinspires.ftc.teamcode.robots.Robot

@Autonomous(name = "Autonomous", group = "Automatic Autonomous")
class RelicRecoveryAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val allianceColor = RelicRecoveryRobot.getAllianceColor(hardwareMap)

        when (allianceColor) {
            Robot.AllianceColor.BLUE ->
                AutoTransitioner.transitionOnStop(this, BlueAutonomous.OPMODE_NAME)

            Robot.AllianceColor.RED ->
                AutoTransitioner.transitionOnStop(this, "Red Autonomous")

            Robot.AllianceColor.UNKNOWN -> {
                telemetry.addLine("Could Not Determine Alliance Color.")
                telemetry.update()
            }
        }

        requestOpModeStop()
    }

}
