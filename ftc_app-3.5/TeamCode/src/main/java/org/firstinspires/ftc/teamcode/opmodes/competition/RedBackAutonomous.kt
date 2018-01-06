package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.libraries.vision.PictographIdentifier
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import kotlin.concurrent.thread

@Autonomous(name = "4: Red Back", group = "Red Manual Selection Autonomous")
class RedBackAutonomous : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "4: Red Back"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {

        // Setup the robot.
        val robot = RelicRecoveryRobot()
        robot.linearOpMode = this
        robot.setup(hardwareMap)
        RelicRecoveryRobotOpModeManager.queueOpMode(this, RelicRecoveryTeleOp.OPMODE_NAME)

        // Prepare to find the position of the jewels.
        val jewelDetector = JewelConfigurationDetector()
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())
        jewelDetector.enable()

        // Wait for the gyro to be calibrated.
        robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.CALIBRATING)
        robot.waitForGyroCalibration()
        robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.READY)

        // Wait for the opmode to start.
        waitForStart()

        // Start the timer.
        val elapsedTime = ElapsedTime(ElapsedTime.Resolution.SECONDS)

        // Start the robot.
        robot.start()
        robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.DETECTING)

        // Find the position of the jewels.
        val jewelPosition = jewelDetector.waitForJewelIdentification(elapsedTime, this)
        jewelDetector.disable()

        // Read the pictograph.
        val pictographIdentifier = PictographIdentifier(hardwareMap)
        pictographIdentifier.activate()
        val pictograph = pictographIdentifier.waitForPictographIdentification(elapsedTime, this)
        pictographIdentifier.deactivate()

        when (pictograph) {
            RelicRecoveryVuMark.LEFT -> {
            }
            RelicRecoveryVuMark.CENTER -> {
            }
            RelicRecoveryVuMark.RIGHT -> {
            }
        }

        // Prepare to begin moving.
        robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.RUNNING)
        robot.closeGlyphGrabbers(500)
        thread(start = true) {
            robot.setLiftPosition(RelicRecoveryRobot.AUTO_LIFT_FIRST_LEVEL)
        }

        // Knock off the correct jewel.
        when (jewelPosition) {
            JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELDETECTED)
                robot.lowerJewelStick()
                robot.timeDrive(500, 0.25)
                robot.raiseJewelStick()
                robot.driveOnBalancingStone(-0.25)
                robot.driveOffBalancingStone(-0.15)
            }

            JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELDETECTED)
                robot.lowerJewelStick()
                robot.driveOffBalancingStone(-0.15)
                robot.raiseJewelStick(500)
            }

            JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                robot.driveOffBalancingStone(-0.15)
                robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.JEWELUNKNOWN)
            }
        }

        // Approach the crypto box.
        robot.timeDrive(250, -0.50)
        robot.turn(0.50, 90.0)
        robot.driveToDistanceFromForwardObject(RelicRecoveryRobot.CRYPTO_BOX_SPACING)

        // Determine the distance from the wall to the correct crypto box.
        val rightWallDistance = when (pictograph) {
            RelicRecoveryVuMark.LEFT -> RelicRecoveryRobot.TRAILING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.CENTER -> RelicRecoveryRobot.CENTER_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.RIGHT -> RelicRecoveryRobot.LEADING_SIDE_CRYPTO_BOX_DISTANCE
            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryRobot.LEADING_SIDE_CRYPTO_BOX_DISTANCE
        }

        // Drive to the correct crypto box.
        robot.driveToDistanceFromRightObject(rightWallDistance)

        // Place the glyph in the correct crypto box.
        robot.dropLift()
        robot.timeDrive(1000)
        robot.extendGlyphDeployer()
        robot.openGlyphGrabbers(250)

        // Back away from the crypto box.
        robot.timeDrive(850, -0.25)
        robot.liftGlyphDeployer(500)

        // Turn towards the center glyphs.
        robot.turn(0.50, -90.0)

        // Open the glyph grabbers.
        robot.releaseGlyphGrabbers()
        robot.retractGlyphDeployer()

        // Drive to the glyphs in the center and grab one.
        robot.timeDrive(1250)
        robot.closeGlyphGrabbers(1250)

        // Drive back the crypto boxes.
        robot.timeDrive(750, -0.50 / 2)
        thread(true) { robot.setLiftPosition(RelicRecoveryRobot.AUTO_LIFT_FIRST_LEVEL) }
        robot.turn(0.75 / 2, -270.0)
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

        robot.setColorBeaconState(RelicRecoveryRobot.ColorBeaconState.DONE)
    }
}