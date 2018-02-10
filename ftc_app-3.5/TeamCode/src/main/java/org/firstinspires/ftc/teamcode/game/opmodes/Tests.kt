package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.game.robots.Coda

@Autonomous
class CodaDriveTrainStallTest : LinearOpMode() {
    override fun runOpMode() {

        val robot = Coda(this)
        val timer = ElapsedTime()

        while(opModeIsActive()) {
            val drivePower = timer.milliseconds()/TEST_TIME_SECONDS
            robot.driveTrain.linearDriveAtPower(drivePower)
            if(robot.driveTrain.currentLinearEncoderPosition() > TEST_COMPLETION_THRESHOLD) {
                requestOpModeStop()
            }

            telemetry.addLine("Drive Power: $drivePower")
            telemetry.update()
        }

    }

    companion object {
        private const val TEST_TIME_SECONDS = 50
        private const val TEST_COMPLETION_THRESHOLD = 100
    }

}