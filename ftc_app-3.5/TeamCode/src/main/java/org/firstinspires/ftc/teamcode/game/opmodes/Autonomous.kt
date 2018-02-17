package org.firstinspires.ftc.teamcode.game.opmodes

import OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.teamcode.game.RelicRecoveryConstants
import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.CodaJewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import org.firstinspires.ftc.teamcode.lib.powercontroller.PIDPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.ProportionalPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.StaticPowerController
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.Heading
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.MecanumDriveTrain
import kotlin.concurrent.thread

private class CodaRelicRecoveryAutonomousActions(
    val linearOpMode: LinearOpMode,
    private val allianceColor: AllianceColor,
    private val cryptoBoxPosition: CryptoBoxPosition,
    private val cryptoBoxHeading: Heading,
    private val glyphPitHeading: Heading
) {

    var robot: Coda
        private set
    var detectedJewelConfiguration = JewelConfigurationDetector.JewelConfiguration.UNKNOWN
        private set
    var detectedPictograph = RelicRecoveryVuMark.UNKNOWN
        private set

    val jewelConfigurationDetector by lazy {
        JewelConfigurationDetector(linearOpMode)
    }

    val pictographIdentifier by lazy {
        PictographIdentifier(linearOpMode)
    }

    enum class AllianceColor {
        RED, BLUE
    }

    enum class CryptoBoxPosition {
        FRONT, SIDE
    }

    private val wallDirection: Coda.RangeSensorDirection
        get() = if (allianceColor == AllianceColor.BLUE) {
                Coda.RangeSensorDirection.LEFT
            } else {
                Coda.RangeSensorDirection.RIGHT
            }

    private val wallDistance: Double
        get() = when (cryptoBoxPosition) {
            CryptoBoxPosition.FRONT ->
                when (allianceColor) {
                    AllianceColor.RED ->
                        when (detectedPictograph) {
                            RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
                        }
                    AllianceColor.BLUE ->
                        when (detectedPictograph) {
                            RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_FRONT_CRYPTO_BOX_DISTANCE
                        }
                }


            CryptoBoxPosition.SIDE ->
                when (allianceColor) {
                    AllianceColor.RED ->
                        when (detectedPictograph) {
                            RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_SIDE_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.LEADING_SIDE_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_SIDE_CRYPTO_BOX_DISTANCE
                        }
                    AllianceColor.BLUE ->
                        when (detectedPictograph) {
                            RelicRecoveryVuMark.LEFT -> RelicRecoveryConstants.LEADING_SIDE_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.CENTER -> RelicRecoveryConstants.CENTER_SIDE_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.RIGHT -> RelicRecoveryConstants.TRAILING_SIDE_CRYPTO_BOX_DISTANCE
                            RelicRecoveryVuMark.UNKNOWN -> RelicRecoveryConstants.LEADING_SIDE_CRYPTO_BOX_DISTANCE
                        }
                }

        }

    init {
        robot = Coda(linearOpMode)
        robot.setup()
        OpModeManager.queueOpMode(linearOpMode, "TeleOp")
    }

    private fun deliverGlyph() {
        robot.lift.drop(3000)
        robot.driveTrain.linearTimeDrive(
            1750,
            StaticPowerController(0.3),
            MecanumDriveTrain.DriveDirection.FORWARD
        )
        robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.RELEASE)
        robot.driveTrain.linearEncoderDrive(-200, StaticPowerController(0.25))
        robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.SMALL_OPEN)
    }

    private fun alignWithCryptoBoxColumn() {
        robot.driveToDistanceFromObject(
            wallDirection,
            wallDistance,
            PIDPowerController(
                linearOpMode,
                CRYPTO_BOX_ALIGNMENT_PID_COEFFICIENTS,
                shouldPrintDebug = true
            )
        )
    }

    @Throws(InterruptedException::class)
    fun performSharedActionGroup1() {

        // Setup the jewel configuration detector and pictograph identifier.
        val context = linearOpMode.hardwareMap.appContext
        val viewDisplay = CameraViewDisplay.getInstance()
        if (!linearOpMode.isStopRequested) {
            jewelConfigurationDetector.init(context, viewDisplay)
        }

        // Wait for start.
        linearOpMode.waitForStart()

        if (!linearOpMode.isStopRequested) {

            // Enable the jewel configuration detector.
            if (!linearOpMode.isStopRequested) {
                jewelConfigurationDetector.enable()
            }

            // Grab the glyph.
            val glyphGrabbingThread = thread(start = true) {
                robot.glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.CLOSED)
                linearOpMode.sleep(750)
                robot.lift.position = CodaLift.LiftPosition.FIRST_LEVEL
            }

            // Detect the jewel configuration.
            detectedJewelConfiguration = jewelConfigurationDetector.waitForJewelIdentification()

            // Move the jewel displacement bar if applicable.
            if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
                robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.DOWN)
            }

            jewelConfigurationDetector.disable()
            pictographIdentifier.activate()

            // Detect the pictograph.
            detectedPictograph = pictographIdentifier.waitForPictographIdentification()
            pictographIdentifier.deactivate()

            // Wait for the grabber to grab the glyph.
            glyphGrabbingThread.join()

            // Knock off the correct jewel.
            when (allianceColor) {
                AllianceColor.RED ->
                    when (detectedJewelConfiguration) {
                        JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                            robot.driveTrain.linearEncoderDrive(300, StaticPowerController(0.175))
                            robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                            robot.driveOnBalancingStone(StaticPowerController(0.40))
                            robot.driveOffBalancingStone(-0.175)
                        }

                        JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                            robot.driveOffBalancingStone(-0.175)
                            robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                        }

                        JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                            robot.driveOffBalancingStone(-0.175)
                        }
                    }

                AllianceColor.BLUE ->
                    when (detectedJewelConfiguration) {
                        JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                            robot.driveTrain.linearEncoderDrive(-300, StaticPowerController(0.175))
                            robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                            robot.driveOnBalancingStone(StaticPowerController(0.40))
                            robot.driveOffBalancingStone(0.175)
                        }

                        JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                            robot.driveOffBalancingStone(0.175)
                            robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                        }

                        JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                            robot.driveOffBalancingStone(0.175)
                        }
                    }
            }
        }

    }

    @Throws(InterruptedException::class)
    fun performSharedActionGroup2() {
        if (!linearOpMode.isStopRequested) {
            if (cryptoBoxPosition == CryptoBoxPosition.FRONT) {
                robot.driveTrain.linearTimeDrive(
                    500,
                    StaticPowerController(0.30),
                    MecanumDriveTrain.DriveDirection.REVERSE
                )
            }

            alignWithCryptoBoxColumn()
            deliverGlyph()

            robot.driveTrain.turnToHeading(
                glyphPitHeading,
                CodaRelicRecoveryAutonomousActions.TURN_POWER_CONTROLLER
            )
        }
    }

    companion object {
        val TURN_POWER_CONTROLLER = ProportionalPowerController(0.0095)
        val CRYPTO_BOX_ALIGNMENT_PID_COEFFICIENTS = {
            val coefficients = PIDCoefficients()
            coefficients.p = 0.0125
            coefficients.i = 0.015
            coefficients
        }()
    }

}

@Autonomous(name = "Blue Front", group = "Gameplay")
class BlueFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val actions = CodaRelicRecoveryAutonomousActions(
            this,
            CodaRelicRecoveryAutonomousActions.AllianceColor.BLUE,
            CodaRelicRecoveryAutonomousActions.CryptoBoxPosition.FRONT,
            CRYPTO_BOX_HEADING,
            GLYPH_PIT_HEADING
        )

        actions.performSharedActionGroup1()
        actions.performSharedActionGroup2()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 0.0
        private const val GLYPH_PIT_HEADING = -155.0
    }

}

@Autonomous(name = "Red Front", group = "Gameplay")
class RedFront : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val actions = CodaRelicRecoveryAutonomousActions(
            this,
            CodaRelicRecoveryAutonomousActions.AllianceColor.RED,
            CodaRelicRecoveryAutonomousActions.CryptoBoxPosition.FRONT,
            CRYPTO_BOX_HEADING,
            GLYPH_PIT_HEADING
        )

        actions.performSharedActionGroup1()
        actions.robot.driveTrain.turnToHeading(
            CRYPTO_BOX_HEADING,
            CodaRelicRecoveryAutonomousActions.TURN_POWER_CONTROLLER
        )
        actions.performSharedActionGroup2()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 180.0
        private const val GLYPH_PIT_HEADING = -25.0
    }

}

@Autonomous(name = "Blue Side", group = "Gameplay")
class BlueRear : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val actions = CodaRelicRecoveryAutonomousActions(
            this,
            CodaRelicRecoveryAutonomousActions.AllianceColor.BLUE,
            CodaRelicRecoveryAutonomousActions.CryptoBoxPosition.SIDE,
            CRYPTO_BOX_HEADING,
            GLYPH_PIT_HEADING
        )

        actions.performSharedActionGroup1()
        actions.robot.driveTrain.turnToHeading(
            CRYPTO_BOX_HEADING,
            CodaRelicRecoveryAutonomousActions.TURN_POWER_CONTROLLER
        )

        actions.performSharedActionGroup2()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 90.0
        private const val GLYPH_PIT_HEADING = -90.0
    }

}

@Autonomous(name = "Red Side", group = "Gameplay")
class RedRear : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val actions = CodaRelicRecoveryAutonomousActions(
            this,
            CodaRelicRecoveryAutonomousActions.AllianceColor.RED,
            CodaRelicRecoveryAutonomousActions.CryptoBoxPosition.SIDE,
            CRYPTO_BOX_HEADING,
            GLYPH_PIT_HEADING
        )

        actions.performSharedActionGroup1()
        actions.robot.driveTrain.turnToHeading(
            CRYPTO_BOX_HEADING,
            CodaRelicRecoveryAutonomousActions.TURN_POWER_CONTROLLER
        )

        actions.performSharedActionGroup2()
    }

    companion object {
        private const val CRYPTO_BOX_HEADING = 90.0
        private const val GLYPH_PIT_HEADING = -90.0
    }

}
