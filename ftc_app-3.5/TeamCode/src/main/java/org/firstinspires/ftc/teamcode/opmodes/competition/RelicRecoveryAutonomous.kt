package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import org.firstinspires.ftc.teamcode.robots.Robot

@Autonomous(name = "Automatically Select", group = "Automatically Select")
class RelicRecoveryAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        val setupPosition = RelicRecoveryRobot().getRobotSetupPosition(hardwareMap)

        when(setupPosition) {
            Robot.SetupPosition.FRONT_BLUE ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, BlueFrontAutonomous.OPMODE_NAME)

            Robot.SetupPosition.BACK_BLUE ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, BlueBackAutonomous.OPMODE_NAME)

            Robot.SetupPosition.FRONT_RED ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, RedFrontAutonomous.OPMODE_NAME)

            Robot.SetupPosition.BACK_RED ->
                RelicRecoveryRobotOpModeManager.queueOpMode(this, RedBackAutonomous.OPMODE_NAME)

            Robot.SetupPosition.UNKNOWN -> {
                telemetry.addLine("-!!!| Could Not Determine Alliance Color! |!!!-")
                telemetry.update()
                waitForStart(); while(opModeIsActive()){}
            }
        }

        requestOpModeStop()
    }

}
