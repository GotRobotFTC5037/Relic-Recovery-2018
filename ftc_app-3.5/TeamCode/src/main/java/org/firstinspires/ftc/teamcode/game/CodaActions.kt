package org.firstinspires.ftc.teamcode.game

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.JewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import java.security.InvalidParameterException
import kotlin.concurrent.thread

class CodaActions(private val linearOpMode: LinearOpMode, private val robot: Coda) {

    private lateinit var jewelConfigurationDetector: JewelConfigurationDetector
    private lateinit var pictographIdentifier: PictographIdentifier

    private lateinit var elevateGlyphThread: Thread
    private lateinit var autonomousTimer: ElapsedTime

    private lateinit var detectedJewelConfiguration: JewelConfigurationDetector.JewelConfiguration

    lateinit var detectedPictograph: RelicRecoveryVuMark
        private set

    fun setupCameras() {
        jewelConfigurationDetector = JewelConfigurationDetector()
        pictographIdentifier = PictographIdentifier(linearOpMode.hardwareMap)
    }

    private fun startAutonomousTimer() {
        autonomousTimer = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)
    }

    fun waitForStart() {
        linearOpMode.waitForStart()
        startAutonomousTimer()
    }

    fun elevateGlyph() {
        elevateGlyphThread = thread(true) {
            robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED, 1100)
            thread(true) {
                //robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
            }
        }
    }

    private fun dropJewelStickIfAppropriate() {
        if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.DOWN)
        }
    }

    fun performCameraIdentificationActions() {
        detectedJewelConfiguration = jewelConfigurationDetector.waitForJewelIdentification(autonomousTimer, linearOpMode)
        jewelConfigurationDetector.disable()

        dropJewelStickIfAppropriate()

        pictographIdentifier.activate()
        detectedPictograph = pictographIdentifier.waitForPictographIdentification(autonomousTimer, linearOpMode)
        pictographIdentifier.deactivate()
    }

    fun waitForGlyphElevation() {
        while (elevateGlyphThread.isAlive) {
            linearOpMode.sleep(10)
        }
    }

    private fun performRetroJewelManeuverIfAppropriate(backwards: Boolean) {
        if (detectedJewelConfiguration == JewelConfigurationDetector.JewelConfiguration.RED_BLUE) {
            performRetroJewelDisplacementManeuver(backwards)
        }
    }

    fun setLiftPositionToFirstLevel() {
        thread(true) {
            //robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL, 0.3)
        }
    }

    enum class AllianceColor {
        RED,
        BLUE
    }

    fun displaceJewel(allianceColor: AllianceColor) {
        when (allianceColor) {
            AllianceColor.RED -> {
                performRetroJewelManeuverIfAppropriate(false)
                robot.driveOffBalancingStone(-0.175)
            }

            AllianceColor.BLUE -> {
                performRetroJewelManeuverIfAppropriate(true)
                robot.driveOffBalancingStone(0.175)
            }
        }
    }

    private fun performRetroJewelDisplacementManeuver(backwards: Boolean) {
        val powerMultiplier = if (backwards) -1 else 1

        robot.driveTrain.linearTimeDrive(1000, 0.175 * powerMultiplier)
        robot.jewelDisplacementBar.setPosition(JewelDisplacementBar.Position.UP)
        robot.driveOnBalancingStone(-0.40 * powerMultiplier)
    }

    fun placeGlyph() {
        robot.driveTrain.linearTimeDrive(1000, 1.0)
        robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.OPEN, 200)
        robot.driveTrain.linearTimeDrive(450, -0.50)
    }

    fun placeGlyph(wallDirection: Coda.ObjectDirection, wallDistance: Double) {
        if (!(wallDirection == Coda.ObjectDirection.LEFT || wallDirection == Coda.ObjectDirection.RIGHT)) {
            throw InvalidParameterException()
        }

        robot.driveToDistanceFromObject(wallDirection, wallDistance)
        //robot.lift.drop()
        placeGlyph()
        robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.OPEN)
    }

    fun driveToTrailingCryptoBox() {
        robot.driveToDistanceFromObject(
            Coda.ObjectDirection.LEFT,
            RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE,
            1.0, false
        )
    }

    enum class DistanceFromCenter(val duration: Long) {
        SHORT(500),
        LONG(700)
    }

    fun grabSecondGlyph(distanceFromCenter: DistanceFromCenter) {
        linearOpMode.sleep(1000)
        robot.driveTrain.linearTimeDrive(distanceFromCenter.duration, 1.0)
        robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.CLOSED)
        robot.driveTrain.linearTimeDrive(distanceFromCenter.duration, -0.25)
        thread(true) {
            //robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL)
        }
    }

    fun driveBackToCryptoBoxFromCenter() {
        robot.driveTrain.linearTimeDrive(500, 1.0)
        robot.driveToDistanceFromObject(Coda.ObjectDirection.FRONT_LEFT, 50.0)
        robot.driveToDistanceFromObject(
            Coda.ObjectDirection.LEFT,
            RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
        )
    }

}