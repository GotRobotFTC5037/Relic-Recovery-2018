package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabbers
import org.firstinspires.ftc.teamcode.game.components.JewelStick
import org.firstinspires.ftc.teamcode.game.components.Lift
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.libraries.RobotOpMode
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.libraries.vision.PictographIdentifier
import kotlin.concurrent.thread

@Autonomous(name = "1: Blue Front", group = "Blue Manual Selection Autonomous")
class BlueFrontAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "1: Blue Front"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Setup the robot.
        val robot = Coda(this)
        val jewelConfigurationDetector = JewelConfigurationDetector()
        val pictographIdentifier = PictographIdentifier(hardwareMap)

        // Wait for the opmode to start.
        waitForStart()

        jewelConfigurationDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())

        // Grab the glyph.
        val glyphGrabbingThread = thread(true) {
            robot.glyphGrabbers.setGlyphGrabberState(GlyphGrabbers.GlyphGrabberState.CLOSED, 1100)
            thread(true) {
                robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
            }
        }

        // Start the timer.
        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

        // Find the position of the jewels.
        val jewelPosition = jewelConfigurationDetector.waitForJewelIdentification(elapsedTime, this)
        jewelConfigurationDetector.disable()

        // Lower the jewel stick as soon as we determine the jewel configuration.
        if (jewelPosition != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelStick.setPosition(JewelStick.Position.DOWN)
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
                robot.driveTrain.timeDrive(1000, -0.175)
                robot.jewelStick.setPosition(JewelStick.Position.UP)
                robot.driveOnBalancingStone()
                robot.driveOffBalancingStone(0.175)
                //robot.driveTrain.powerBreak()
            }

            JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                robot.driveOffBalancingStone(0.175)
                //robot.driveTrain.powerBreak()
                robot.jewelStick.setPosition(JewelStick.Position.UP)
            }

            JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                robot.driveOffBalancingStone(0.175)
                //robot.driveTrain.powerBreak()
            }
        }

        thread(true) {
            robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL, 0.3)
        }

        // Determine the distance from the wall to the correct crypto box.
        val leftWallDistance = when (pictograph) {
            RelicRecoveryVuMark.LEFT -> Coda.LEADING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> Coda.CENTER_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> Coda.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> Coda.LEADING_FRONT_CRYPTO_BOX_DISTANCE
        }

        // Drive to the correct crypto box.
        robot.driveTrain.timeDrive(750, -0.225)
        robot.driveToDistanceFromObject(Coda.ObjectDirection.LEFT, leftWallDistance)

        // Place the glyph in the correct crypto box.
        robot.lift.drop()
        robot.driveTrain.timeDrive(1000)
        robot.glyphGrabbers.setGlyphGrabberState(GlyphGrabbers.GlyphGrabberState.RELEASE, 250)

        // Back away from the crypto box.
        robot.driveTrain.timeDrive(1250, -0.225)
        robot.glyphGrabbers.setGlyphGrabberState(GlyphGrabbers.GlyphGrabberState.OPEN)

        // Turn towards the center glyphs.
        if (pictograph != RelicRecoveryVuMark.RIGHT) {
            robot.driveToDistanceFromObject(Coda.ObjectDirection.LEFT, Coda.TRAILING_FRONT_CRYPTO_BOX_DISTANCE, 1.0, false)
        }

        robot.driveTrain.turnTo(0.20, -135.0)

        // Drive to the glyphs in the center and grab one.
        sleep(1000)
        robot.driveTrain.timeDrive(700, 1.0)
        robot.glyphGrabbers.setGlyphGrabberState(GlyphGrabbers.GlyphGrabberState.CLOSED)

        // Drive back the crypto boxes.
        robot.driveTrain.timeDrive(650, -0.25)
        thread(true) { robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL) }
        robot.driveTrain.turnTo(0.35, 0.0)
        robot.driveTrain.timeDrive(500)
        //robot.driveToDistanceFromForwardObject(Coda.CRYPTO_BOX_SPACING)

        // Line up with the center crypto box.
        //robot.driveToDistanceFromLeftObject(Coda.TRAILING_FRONT_CRYPTO_BOX_DISTANCE)

        // Place the glyph in the correct crypto box.
        if (pictograph == RelicRecoveryVuMark.RIGHT) {
            robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
        } else {
            robot.lift.drop()
        }

        robot.driveTrain.timeDrive(1000)
        robot.glyphGrabbers.setGlyphGrabberState(GlyphGrabbers.GlyphGrabberState.OPEN, 200)

        // Back away from the crypto box.
        robot.driveTrain.timeDrive(450, -0.50)
        //robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.driveTrain.turnTo(0.35, -135.0)
    }

}
