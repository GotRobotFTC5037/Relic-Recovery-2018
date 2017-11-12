package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.AutoTransitioner
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Red Autonomous", group = "Manual Selection Autonomous")
class RedAutonomous : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        // Setup the robot
        val robot = RelicRecoveryRobot
        robot.linearOpMode = this
        RelicRecoveryRobot.setup(hardwareMap)

        // TODO: Detect which crypto box is the bonus one.

        // Prepare for TeleOp transition after Autonomous is run.
        AutoTransitioner.transitionOnStop(this, "TeleOp")

        // Wait for the imu gyro to calibrate.
        robot.waitForGyroCalibration()
        RelicRecoveryRobot.colorBeacon.green()

        // Wait for the OpMode to start.
        waitForStart()
        RelicRecoveryRobot.colorBeacon.blue()

        // Close the Glyph Grabbers in order to grab the glyph in front of us as well as
        // to Allow the front Ultrasonic Sensor to see in front of them.
        RelicRecoveryRobot.closeGlyphGrabbers()
        sleep(1000)

        // TODO: Add back the jewel knocking code.

        // Drive until 45cm away from the wall.
        RelicRecoveryRobot.driveToDistanceFromForwardObject(45.0, 0.25)
        robot.stopAllDriveMotors()
        sleep(1000)

        // Drive to the correct glyph position.
        // TODO: Move the robot to the correct crypto box.
        robot.setStrafePower(0.75)
        sleep(2500)
        robot.stopAllDriveMotors()

        // TODO: Drive the robot forward to deliver the crypto box.

        // TODO: Drive the robot backwards to stop touching the crypto box.
    }

}