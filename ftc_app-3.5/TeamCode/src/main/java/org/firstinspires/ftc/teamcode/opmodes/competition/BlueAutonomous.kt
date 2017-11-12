package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.AutoTransitioner
import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Blue Autonomous", group = "Manual Selection Autonomous")
class BlueAutonomous: LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        // Setup the robot
        val robot = RelicRecoveryRobot
        robot.linearOpMode = this
        robot.setup(hardwareMap)

        // Activate the pictograph identifier.
        val pictographIdentifier = PictographIdentifier()
        pictographIdentifier.activate()

        // Prepare for TeleOp transition after Autonomous is run.
        AutoTransitioner.transitionOnStop(this, "TeleOp")

        // Wait for the imu gyro to calibrate.
        robot.waitForGyroCalibration()
        robot.colorBeacon.green()

        // Wait for the OpMode to start.
        waitForStart()
        robot.colorBeacon.blue()

        // If we haven't identified the pictograph by now, we never will.
        // TODO: Well, unless we will. We should add a feature that makes a callback to a a delegated method if the pictograph ends up being identified.
        // TODO: If the time between the pictograph identifier activation and now is very short, wait a little before getting this value.
        val pictograph = pictographIdentifier.getIdentifyedPictograph()

        // Close the Glyph Grabbers in order to grab the glyph in front of us as well as
        // to Allow the front Ultrasonic Sensor to see in front of them.
        robot.closeGlyphGrabbers()
        sleep(1000)

        // TODO: Add back the jewel knocking code.

        // Drive until 45cm away from the wall.
        robot.driveToDistanceFromForwardObject(45.0, 0.25)
        robot.stopAllDriveMotors()
        sleep(1000)

        // TODO: Find the real values for each crypto box
        val leftWallDistance = when(pictograph) {
            RelicRecoveryVuMark.LEFT -> 0.0
            RelicRecoveryVuMark.CENTER -> 0.0
            RelicRecoveryVuMark.RIGHT -> 0.0
            RelicRecoveryVuMark.UNKNOWN -> 0.0 // TODO: Pick a random crypto box
        }

        // Drive to the correct glyph position.
        // TODO: Move the robot to the correct crypto box.

        // TODO: Drive the robot forward to deliver the crypto box.
        robot.openGlyphGrabbers()
        sleep(500)

        robot.driveTo(0.0, -50.0)
    }

}