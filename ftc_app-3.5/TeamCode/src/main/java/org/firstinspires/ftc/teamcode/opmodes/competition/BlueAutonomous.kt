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
        robot.shouldCorrectHeading = true

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
        val pictograph = pictographIdentifier.getIdentifiedPictograph()
        pictographIdentifier.deactivate()

        // Close the Glyph Grabbers in order to grab the glyph in front of us as well as
        // to allow the front Ultrasonic Sensor to see in front of them.
        robot.closeGlyphGrabbers(); sleep(1000)

        // Lift the lift so that the glyph doesn't drag on the floor.
        robot.setLiftPosition(350, 0.10)

        // TODO: Add back the jewel knocking code.

        // Drive until close enough to the crypto boxes.
        robot.driveToDistanceFromForwardObject(40.0, 0.15)
        sleep(1000)

        // Select the distance from the wall baaed on the pictograph.
        val leftWallDistance = when(pictograph) {
            RelicRecoveryVuMark.LEFT -> 50.0
            RelicRecoveryVuMark.CENTER -> 70.0
            RelicRecoveryVuMark.RIGHT -> 90.0
            RelicRecoveryVuMark.UNKNOWN -> 70.0 // TODO: Pick a random crypto box here.
        }

        // Drive to the correct glyph position.
        robot.driveToDistanceFromLeftObject(leftWallDistance, 0.75)

        // Lower the lift.
        robot.setLiftPosition(0, 0.10)

        // Drop the glyph.
        robot.openGlyphGrabbers(); sleep(500)

        // Push the glyph into the crypto box.
        robot.setDrivePower(0.25); sleep(500)
        robot.stopAllDriveMotors()

        // Back away from the crypto box.
        robot.setDrivePower(-0.25); sleep(200)
    }

}