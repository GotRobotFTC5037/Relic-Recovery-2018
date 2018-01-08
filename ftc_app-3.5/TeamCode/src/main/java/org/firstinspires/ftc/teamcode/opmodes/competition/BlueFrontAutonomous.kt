package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.components.Lift.RelicRecoveryLift
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.libraries.vision.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import kotlin.concurrent.thread

@Autonomous(name = "1: Blue Front", group = "Blue Manual Selection Autonomous")
class BlueFrontAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "1: Blue Front"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        // Setup the robot.
        val robot = RelicRecoveryRobot()
        robot.prepareForAutonomous(this)
        val pictographIdentifier = PictographIdentifier(hardwareMap)

        // Wait for the opmode to start.
        waitForStart()

        // Start the robot.
        robot.start()

        // Grab the glyph.
        robot.closeGlyphGrabbers(1200)
        robot.lift.setPosition(RelicRecoveryLift.LiftPosition.SECOND_LEVEL)

        // Start the timer.
        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

        // Find the position of the jewels.
        val jewelPosition = robot.jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, this)
        robot.jewelConfigurationDetector.disable()

        if (jewelPosition != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.lowerJewelStick(0)
        }

        // Read the pictograph.
        pictographIdentifier.activate()
        val pictograph = pictographIdentifier.waitForPictographIdentification(elapsedTime, this)
        pictographIdentifier.deactivate()

        // Knock off the correct jewel.
        when (jewelPosition) {
            JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                robot.timeDrive(1000, -0.175)
                robot.raiseJewelStick()
                robot.driveOnBalancingStone()
                robot.driveOffBalancingStone(0.175)
                robot.powerBreak()
            }

            JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                robot.driveOffBalancingStone(0.175)
                robot.powerBreak()
                robot.raiseJewelStick()
            }

            JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                robot.driveOffBalancingStone(0.175)
                robot.powerBreak()
            }
        }

        // Approach the crypto box.
        // robot.driveToDistanceFromForwardObject(RelicRecoveryRobot.CRYPTO_BOX_SPACING)

        // Determine the distance from the wall to the correct crypto box.
        val rightWallDistance = when (pictograph) {
            RelicRecoveryVuMark.LEFT -> RelicRecoveryRobot.LEADING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> RelicRecoveryRobot.CENTER_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RelicRecoveryRobot.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryRobot.LEADING_FRONT_CRYPTO_BOX_DISTANCE
        }

        // Drive to the correct crypto box.
        robot.driveToDistanceFromLeftObject(rightWallDistance)

        // Place the glyph in the correct crypto box.
        robot.lift.drop()
        robot.timeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.timeDrive(1250, -0.175)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turn(0.50, -135.0)

        // Open the glyph grabbers.
        robot.releaseGlyphGrabbers()
        robot.retractGlyphDeployer()

        // Drive to the glyphs in the center and grab one.
        robot.timeDrive(2500)
        robot.closeGlyphGrabbers(1250)

        // Drive back the crypto boxes.
        robot.timeDrive(750, -0.25)
        thread(true) { robot.lift.setPosition(RelicRecoveryLift.LiftPosition.FIRST_LEVEL) }
        robot.turn(0.75, 0.0)
        robot.timeDrive(500)
        robot.driveToDistanceFromForwardObject(RelicRecoveryRobot.CRYPTO_BOX_SPACING)

        // Line up with the center crypto box.
        robot.driveToDistanceFromLeftObject(RelicRecoveryRobot.TRAILING_FRONT_CRYPTO_BOX_DISTANCE)

        // Place the glyph in the correct crypto box.
        robot.lift.drop()
        robot.timeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.timeDrive(450, -0.50)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turn(0.25, -135.0)
    }

}
