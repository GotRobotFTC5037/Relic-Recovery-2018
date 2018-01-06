package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.libraries.vision.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import kotlin.concurrent.thread

@Autonomous(name = "2: Blue Back", group = "Blue Manual Selection Autonomous")
class BlueBackAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "2: Blue Back"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        // Setup the robot.
        val robot = RelicRecoveryRobot()
        robot.prepareForAutonomous(this)
        val pictographIdentifier = PictographIdentifier(hardwareMap)

        // Wait for the opmode to start.
        waitForStart()

        // Start the timer.
        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.SECONDS)

        // Start the robot.
        robot.start()

        thread(start = true) {
            robot.closeGlyphGrabbers(750)
            robot.setLiftPosition(RelicRecoveryRobot.AUTO_LIFT_FIRST_LEVEL)
        }

        // Find the position of the jewels.
        val jewelPosition = robot.jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, this)
        robot.jewelConfigurationDetector.disable()
        //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELDETECTED)

        if (jewelPosition != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.lowerJewelStick(0)
            //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELUNKNOWN)
        }

        // Read the pictograph.
        pictographIdentifier.activate()
        val pictograph = pictographIdentifier.waitForPictographIdentification(elapsedTime, this)
        pictographIdentifier.deactivate()
<<<<<<< HEAD
        //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.DETECTING)
=======
>>>>>>> 9a200b1aac761a00a25e54f8b71f515ad8c750e8

        when (pictograph) {
            //RelicRecoveryVuMark.LEFT -> {}
            //RelicRecoveryVuMark.CENTER -> {}
            //RelicRecoveryVuMark.RIGHT -> {}
        }
        // Knock off the correct jewel.
        when (jewelPosition) {
            JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELDETECTED)
                robot.timeDrive(1000, -0.25 / 2)
                robot.raiseJewelStick()
                robot.driveOnBalancingStone(0.50 / 2)
                robot.driveOffBalancingStone(0.15 / 2)
            }

            JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELDETECTED)
                robot.driveOffBalancingStone(0.15 / 2)
                robot.raiseJewelStick()
            }

            JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                robot.driveOffBalancingStone(0.15 / 2)
                //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELUNKNOWN)
            }
        }

        // Approach the crypto box.
        robot.turn(0.50, 90.0)
        robot.driveToDistanceFromForwardObject(RelicRecoveryRobot.CRYPTO_BOX_SPACING)

        // Determine the distance from the wall to the correct crypto box.
        val rightWallDistance = when (pictograph) {
            RelicRecoveryVuMark.LEFT -> RelicRecoveryRobot.LEADING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> RelicRecoveryRobot.CENTER_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RelicRecoveryRobot.TRAILING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryRobot.LEADING_SIDE_CRYPTO_BOX_DISTANCE
        }

        // Drive to the correct crypto box.
        robot.driveToDistanceFromLeftObject(rightWallDistance)

        // Place the glyph in the correct crypto box.
        robot.dropLift()
        robot.timeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.timeDrive(850, -0.25 / 2)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turn(0.90 / 2, -90.0)

        // Open the glyph grabbers.
        robot.releaseGlyphGrabbers()
        robot.retractGlyphDeployer()

        // Drive to the glyphs in the center and grab one.
        robot.timeDrive(1250)
        robot.closeGlyphGrabbers(1250)

        // Drive back the crypto boxes.
        robot.timeDrive(750, -0.50 / 2)
        thread(true) { robot.setLiftPosition(RelicRecoveryRobot.AUTO_LIFT_FIRST_LEVEL) }
        robot.turn(0.75 / 2, 90.0)
        robot.driveToDistanceFromForwardObject(RelicRecoveryRobot.CRYPTO_BOX_SPACING + 25)

        // Line up with the center crypto box.
        robot.driveToDistanceFromLeftObject(RelicRecoveryRobot.CENTER_SIDE_CRYPTO_BOX_DISTANCE, 0.65 / 2)

        // Place the glyph in the correct crypto box.
        robot.dropLift()
        robot.timeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.timeDrive(850, -0.25 / 2)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turn(0.50 / 2, -90.0)

        //robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.DONE)
    }

}