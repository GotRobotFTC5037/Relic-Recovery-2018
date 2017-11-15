package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.AutoTransitioner
import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Blue Autonomous", group = "Manual Selection Autonomous")
class BlueAutonomous: LinearOpMode() {

    companion object {
        val OPMODE_NAME = "Blue Autonomous"

        val DRIVE_POWER = 0.15
        val STRAFE_POWER = 0.75

        val FRONT_WALL_DISTANCE = 40.0
        val LEFT_CRYPTO_BOX_DISTANCE = 40.0
        val CENTER_CRYPTO_BOX_DISTANCE = 63.0
        val RIGHT_CRYPTO_BOX_DISTANCE = 80.0
    }

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
        AutoTransitioner.transitionOnStop(this, RelicRecoveryTeleOp.OPMODE_NAME)

        // Wait for the imu gyro to calibrate.
        robot.waitForGyroCalibration()
        robot.colorBeacon.green()

        // Wait for the OpMode to start.
        waitForStart()
        robot.colorBeacon.blue()

        robot.start()

        // If we haven't identified the pictograph by now, we never will.
        val pictograph = pictographIdentifier.getIdentifiedPictograph()
        pictographIdentifier.deactivate()

        // Close the Glyph Grabbers in order to grab the glyph in front of us as well as
        // to allow the front Ultrasonic Sensor to see in front of them.
        robot.closeGlyphGrabbers(); sleep(2000)

        // Lift the lift so that the glyph doesn't drag on the floor.
        robot.setLiftPosition(RelicRecoveryRobot.LIFT_FIRST_LEVEL)

        // TODO: Add back the jewel knocking code.

        // Drive until close enough to the crypto boxes.
        robot.driveToDistanceFromForwardObject(FRONT_WALL_DISTANCE, DRIVE_POWER)
        sleep(1000)

        // Select the distance from the wall baaed on the pictograph.
        val leftWallDistance = when(pictograph) {
            RelicRecoveryVuMark.LEFT -> LEFT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> CENTER_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RIGHT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> CENTER_CRYPTO_BOX_DISTANCE // TODO: Pick a random crypto box.
        }

        // Drive to the correct glyph position.
        robot.driveToDistanceFromLeftObject(leftWallDistance, STRAFE_POWER)

        // Drop the glyph.
        robot.setLiftPosition(0)
        robot.openGlyphGrabbers()
        sleep(500)

        // Push the glyph into the crypto box.
        robot.setDrivePower(DRIVE_POWER); sleep(750)
        robot.stopAllDriveMotors()

        // Back away from the crypto box.
        robot.setDrivePower(-DRIVE_POWER); sleep(200)
    }

}