package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "Automatically Select", group = "Automatically Select")
@Disabled
class AutoSelectingAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        /*

        val setupPosition = Coda.getRobotSetupPosition(this)

        when(setupPosition) {
            Coda.SetupPosition.FRONT_BLUE ->
                OpModeManager.queueOpMode(this, BlueFrontAutonomous.OPMODE_NAME)

            Coda.SetupPosition.BACK_BLUE ->
                OpModeManager.queueOpMode(this, BlueBackAutonomous.OPMODE_NAME)

            Coda.SetupPosition.FRONT_RED ->
                OpModeManager.queueOpMode(this, RedFrontAutonomous.OPMODE_NAME)

            Coda.SetupPosition.BACK_RED ->
                OpModeManager.queueOpMode(this, RedBackAutonomous.OPMODE_NAME)

            Coda.SetupPosition.UNKNOWN -> {
                telemetry.addLine("-!!!| Could Not Determine Alliance Color! |!!!-")
                telemetry.update()
                waitForStart(); while(opModeIsActive()){}
            }
        }

        */

        requestOpModeStop()
    }
}
