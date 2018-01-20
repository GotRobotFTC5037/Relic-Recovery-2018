package org.firstinspires.ftc.teamcode.game

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabbers
import org.firstinspires.ftc.teamcode.game.components.JewelStick
import org.firstinspires.ftc.teamcode.game.components.Lift
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.libraries.vision.PictographIdentifier
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

    enum class AllianceColor {
        RED,
        BLUE
    }

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
            robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.CLOSED, 1100)
            thread(true) {
                robot.lift.setPosition(Lift.LiftPosition.SECOND_LEVEL)
            }
        }
    }

    private fun dropJewelStickIfAppropriate() {
        if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
            robot.jewelStick.setPosition(JewelStick.Position.DOWN)
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
            robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL, 0.3)
        }
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

        robot.driveTrain.timeDrive(1000, 0.175 * powerMultiplier)
        robot.jewelStick.setPosition(JewelStick.Position.UP)
        robot.driveOnBalancingStone(-0.40 * powerMultiplier)
    }

    fun placeGlyph() {
        robot.driveTrain.timeDrive(1000)
        robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.OPEN, 200)
        robot.driveTrain.timeDrive(450, -0.50)
    }

    fun placeGlyph(wallDirection: Coda.ObjectDirection, wallDistance: Double) {
        if (!(wallDirection == Coda.ObjectDirection.LEFT || wallDirection == Coda.ObjectDirection.RIGHT)) {
            throw InvalidParameterException()
        }

        robot.driveToDistanceFromObject(wallDirection, wallDistance)
        robot.lift.drop()
        placeGlyph()
        robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.OPEN)
    }

    fun driveToTrailingCryptoBox() {
        robot.driveToDistanceFromObject(Coda.ObjectDirection.LEFT,
                Coda.TRAILING_FRONT_CRYPTO_BOX_DISTANCE, 1.0, false)

    }

    enum class DistanceFromCenter(val duration: Long) {
        SHORT(500),
        LONG(700)
    }

    fun grabSecondGlyph(distanceFromCenter: DistanceFromCenter) {
        linearOpMode.sleep(1000)
        robot.driveTrain.timeDrive(distanceFromCenter.duration, 1.0)
        robot.glyphGrabbers.setState(GlyphGrabbers.GlyphGrabberState.CLOSED)
        robot.driveTrain.timeDrive(distanceFromCenter.duration, -0.25)
        thread(true) {
            robot.lift.setPosition(Lift.LiftPosition.FIRST_LEVEL)
        }
    }

    fun driveBackToCryptoBoxFromCenter() {
        robot.driveTrain.timeDrive(500)
        robot.driveToDistanceFromObject(Coda.ObjectDirection.FRONT_LEFT, Coda.CRYPTO_BOX_SPACING)
        robot.driveToDistanceFromObject(Coda.ObjectDirection.LEFT, Coda.TRAILING_FRONT_CRYPTO_BOX_DISTANCE)
    }


}