package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Automatically Select", group = "Automatically Select")
@Disabled
class RelicRecoveryAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        /*

        val setupPosition = RelicRecoveryRobot.getRobotSetupPosition(this)

        when(setupPosition) {
            RelicRecoveryRobot.SetupPosition.FRONT_BLUE ->
                OpModeManager.queueOpMode(this, BlueFrontAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.BACK_BLUE ->
                OpModeManager.queueOpMode(this, BlueBackAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.FRONT_RED ->
                OpModeManager.queueOpMode(this, RedFrontAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.BACK_RED ->
                OpModeManager.queueOpMode(this, RedBackAutonomous.OPMODE_NAME)

            RelicRecoveryRobot.SetupPosition.UNKNOWN -> {
                telemetry.addLine("-!!!| Could Not Determine Alliance Color! |!!!-")
                telemetry.update()
                waitForStart(); while(opModeIsActive()){}
            }
        }

        */

        requestOpModeStop()
    }
}
