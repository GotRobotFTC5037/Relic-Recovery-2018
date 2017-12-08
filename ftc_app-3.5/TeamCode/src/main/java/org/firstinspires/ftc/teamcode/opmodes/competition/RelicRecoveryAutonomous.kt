package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Automatically Select", group = "Automatically Select")
@Disabled
class RelicRecoveryAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val setupPosition = RelicRecoveryRobot.getRobotSetupPosition(hardwareMap)

        when(setupPosition) {
            RelicRecoveryRobot.SetupPosition.FRONT_BLUE ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, BlueFrontAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.BACK_BLUE ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, BlueBackAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.FRONT_RED ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, RedFrontAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.BACK_RED ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, RedBackAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.UNKNOWN -> {
                telemetry.addLine("-!!!| Could Not Determine Alliance Color! |!!!-")
                telemetry.update()
                waitForStart(); while(opModeIsActive()){}
            }
        }

        requestOpModeStop()
    }

}
