package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

@Autonomous(name = "4: Red Back", group = "Red Manual Selection Autonomous")
@Disabled
class RedBackAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "4: Red Back"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        /*
        // Setup the robot.
        val robot = Coda()
        robot.prepareForAutonomous(this)
        val pictographIdentifier = PictographIdentifier(hardwareMap)

        // Wait for the opmode to start.
        waitForStart()

        // Start the robot.
        robot.startAuto()

        // Grab the glyph.
        val glyphGrabbingThread = thread(true) {
            robot.closeGlyphGrabbers(1100)
            thread(true) {
                robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
            }
        }

        // Start the timer.
        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

        // Find the position of the jewels.
        val jewelPosition = robot.jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, this)
        robot.jewelConfigurationDetector.disable()

        // Lower the jewel stick as soon as we determine the jewel configuration.
        if (jewelPosition != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.lowerJewelStick(0)
        }

        // Read the pictograph.
        pictographIdentifier.activate()
        val pictograph = pictographIdentifier.waitForPictographIdentification(elapsedTime, this)
        pictographIdentifier.deactivate()

        // Wait for the glyph to be grabbed.
        // (I find it funny that the robot identifies the jewels *and* pictograph
        // before it even has time to grab the glyph that is right in front of it)
        while (glyphGrabbingThread.isAlive) {
            sleep(10)
        }

        // Knock off the correct jewel.
        when (jewelPosition) {
            JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                robot.linearTimeDrive(1000, 0.175)
                robot.raiseJewelStick()
                robot.driveOnBalancingStone(-0.40)
                robot.driveOffBalancingStone(-0.175)
                robot.powerBreak(false)
            }

            JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                robot.driveOffBalancingStone(-0.175)
                robot.powerBreak(false)
                robot.raiseJewelStick()
            }

            JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                robot.driveOffBalancingStone(-0.175)
                robot.powerBreak(false)
            }
        }

        thread(true) {
            robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL, 0.3)
        }

        // Determine the distance from the wall to the correct crypto box.
        val rightWallDistance = when (pictograph) {
            RelicRecoveryVuMark.LEFT -> Coda.TRAILING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> Coda.CENTER_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> Coda.LEADING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> Coda.LEADING_SIDE_CRYPTO_BOX_DISTANCE
        }

        // Drive to the correct crypto box.
        robot.turnAtPower(0.3, 90.0)
        robot.driveToDistanceFromRightObject(rightWallDistance)

        // Place the glyph in the correct crypto box.
        robot.lift.drop()
        robot.linearTimeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.linearTimeDrive(1250, -0.225)
        robot.liftGlyphDeployer(500)

        // Open the glyph grabbers.
        robot.releaseGlyphGrabbers()
        robot.retractGlyphDeployer()

        robot.turnAtPower(0.20, -90.0)

        // Drive to the glyphs in the center and grab one.
        sleep(1000)
        robot.linearTimeDrive(475, 1.0)
        robot.closeGlyphGrabbers(1250)

        // Drive back the crypto boxes.
        robot.linearTimeDrive(650, -0.25)
        thread(true) { robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL) }
        robot.turnAtPower(0.35, 90.0)
        robot.linearTimeDrive(500)
        robot.driveToDistanceFromForwardObject(Coda.CRYPTO_BOX_SPACING)

        // Line up with the center crypto box.
        robot.driveToDistanceFromRightObject(Coda.CENTER_SIDE_CRYPTO_BOX_DISTANCE)

        // Place the glyph in the correct crypto box.
        if (pictograph == RelicRecoveryVuMark.CENTER) {
            robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
        } else {
            robot.lift.drop()
        }

        robot.linearTimeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.linearTimeDrive(450, -0.50)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turnAtPower(0.35, -90.0)
        */
    }
}