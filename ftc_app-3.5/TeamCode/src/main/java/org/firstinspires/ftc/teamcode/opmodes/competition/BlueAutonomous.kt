package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.AutoTransitioner
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Blue Autonomous", group = "Manual Autonomous")
class BlueAutonomous: LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot
        robot.linearOpMode = this
        robot.setup(hardwareMap)
        AutoTransitioner.transitionOnStop(this, "TeleOp")
        waitForStart()
        robot.waitForGyroCalibration()

        robot.driveUntilThresholdReached(50.0, 0.5)
        robot.setStrafePower(0.75)
        sleep(750)
        robot.stopAllDriveMotors()
    }

}