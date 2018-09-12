package org.firstinspires.ftc.teamcode.game.actions

import OpModeController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.game.RelicRecoveryConstants
import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabbers
import org.firstinspires.ftc.teamcode.game.components.CodaJewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.elements.CryptoBox
import org.firstinspires.ftc.teamcode.game.elements.Glyph
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import org.firstinspires.ftc.teamcode.lib.powercontroller.PIDPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.ProportionalPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.StaticPowerController
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.Heading
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.MecanumDriveTrain
import kotlin.concurrent.thread

/** Runs complex actions on Coda that are primarily autonomous. */
class CodaAutonomousActions(

    /** The linear op mode that the robot is running on. */
    val linearOpMode: LinearOpMode,

    /** The robot to be used to perform actions. */
    val robot: Coda = Coda(linearOpMode)
) {

    /** The alliance color of the robot to be used to perform actions. */
    var allianceColor = AllianceColor.UNDETERMINED

    /** An enum class for the current alliance color. */
    enum class AllianceColor {

        /** Used when the alliance color is red. */
        RED,

        /** Used when the alliance color is blue. */
        BLUE,

        /** Used when the alliance color has not been determined. */
        UNDETERMINED
    }

    /** The position of the target crypto box. */
    var cryptoBoxPosition = CryptoBoxPosition.UNDETERMINED

    /** An enum class for the target crypto box position. */
    enum class CryptoBoxPosition {

        /** Used when the crypto box is facing the audience. */
        FRONT,

        /** Used when the crypto box is facing the alliance team members. */
        SIDE,

        /** Used when the crypto box position has not been determined. */
        UNDETERMINED
    }

    /** The heading of the target crypto box while the robot is at the glyph pit. */
    var cryptoBoxHeading: Heading = 0.0

    /** The heading of the glyph pit while the robot is at the crypto box. */
    var glyphPitHeading: Heading = 0.0

    private val jewelConfigurationDetector by lazy { JewelConfigurationDetector(linearOpMode) }
    private var detectedJewelConfiguration = JewelConfigurationDetector.JewelConfiguration.UNKNOWN

    private val pictographIdentifier by lazy { PictographIdentifier(linearOpMode) }
    private var detectedPictographColumn: CryptoBox.ColumnPosition? = null

    /** Performs all of the actions done inside of our autonomous opmodes. */
    @Throws(InterruptedException::class)
    fun performForAutonomousOpMode() {
        prepareForAutonomousOpMode()
        linearOpMode.waitForStart()

        val glyphGrabbingThread = thread(start = true) {
            robot.glyphGrabber.setState(
                CodaGlyphGrabbers.GlyphGrabberState.CLOSED,
                GLYPH_GRAB_WAIT_TIME
            )
            robot.lift.position = CodaLift.LiftPosition.FIRST_LEVEL
        }

        performCameraIdentifications()

        glyphGrabbingThread.join()
        robot.lift.shouldHoldLiftPosition = false

        displaceIdentifiedJewel()

        if (isNotBlueFront()) turnToCryptoBox()

        robot.lift.resetEncoder()

        distanceFromCryptoBox()

        alignWithColumn(detectedPictographColumn)

        robot.lift.drop()

        deliverGlyph()

        val cryptoBox = CryptoBox().apply {
            val pictographColumn = detectedPictographColumn
                ?: when (allianceColor) {
                    AllianceColor.BLUE -> CryptoBox.ColumnPosition.LEFT
                    AllianceColor.RED -> CryptoBox.ColumnPosition.RIGHT
                    else -> TODO("If this happens, we have bigger problems.")
                }

            addGlyphToColumn(Glyph(Glyph.Color.UNKNOWN), pictographColumn)
        }

        if (SHOULD_GRAB_ADDITIONAL_GLYPHS) {
            repeat(2) {

                val liftDropThread = thread(start = true) {
                    robot.lift.shouldHoldLiftPosition = false
                    robot.lift.drop()
                }

                alignWithGlyphPit()

                turnToGlyphPit()

                robot.glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.SMALL_OPEN)

                robot.driveTrain.linearEncoderDrive(
                    GLYPH_PIT_DISTANCE,
                    StaticPowerController(GLYPH_PIT_DRIVE_POWER)
                )

                liftDropThread.join()

                robot.glyphGrabber.setState(
                    CodaGlyphGrabbers.GlyphGrabberState.CLOSED,
                    GLYPH_GRAB_WAIT_TIME
                )

                robot.lift.shouldHoldLiftPosition = true
                robot.lift.position = CodaLift.LiftPosition.SECOND_LEVEL

                robot.driveTrain.linearEncoderDrive(
                    GLYPH_PIT_REVERSE_DISTANCE,
                    StaticPowerController(GLYPH_PIT_REVERSE_DRIVE_POWER)
                )

                val position = cryptoBox.positionForNextGlyph()?.also {
                    robot.lift.moveToRow(it.row)
                }

                turnToCryptoBox()

                // distanceFromCryptoBox()

                robot.driveTrain.linearEncoderDrive(
                    ADDITIONAL_CRYPTO_BOX_APPROACH_DISTANCE,
                    StaticPowerController(ADDITIONAL_CRYPTO_BOX_APPROACH_DRIVE_POWER)
                )

                position?.let {
                    alignWithColumn(it.column)
                }

                deliverGlyph()

                position?.let {
                    cryptoBox.addGlyphToColumn(Glyph(Glyph.Color.UNKNOWN), it.column)
                }
            }
        }

    }

    private fun distanceFromCryptoBox() {
        robot.driveToDistanceFromObject(
            Coda.RangeSensorDirection.FRONT_LEFT,
            45.0,
            ProportionalPowerController(0.015),
            shouldCorrect = false
        )
    }

    private fun prepareForAutonomousOpMode() {
        robot.setup()

        OpModeController.queueOpMode(linearOpMode, "TeleOp")

        jewelConfigurationDetector.init(
            linearOpMode.hardwareMap.appContext,
            CameraViewDisplay.getInstance()
        )
    }

    private fun performCameraIdentifications() {
        jewelConfigurationDetector.enable()

        detectedJewelConfiguration = jewelConfigurationDetector.waitForJewelIdentification()

        if (jewelConfigurationIsKnown())
            robot.jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.DOWN)

        jewelConfigurationDetector.disable()
        pictographIdentifier.activate()

        detectedPictographColumn = pictographIdentifier.waitForPictographIdentification()
        pictographIdentifier.deactivate()
    }

    private fun displaceIdentifiedJewel() {
        with(robot) {
            val multiplier = if (allianceColor == AllianceColor.BLUE) 1 else -1
            when (detectedJewelConfiguration) {
                JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                    driveTrain.linearEncoderDrive(
                        DRIVE_OFF_BALANCING_STONE_DISTANCE * multiplier * -1,
                        StaticPowerController(DRIVE_OFF_BALANCING_STONE_DRIVE_POWER)
                    )
                    jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                    driveOnBalancingStone(StaticPowerController(DRIVE_ON_BALANCING_STONE_POWER))
                    driveOffBalancingStone(DRIVE_OFF_BALANCING_STONE_DRIVE_POWER * multiplier)
                }

                JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                    driveOffBalancingStone(DRIVE_OFF_BALANCING_STONE_DRIVE_POWER * multiplier)
                    jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                }

                JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                    driveOffBalancingStone(DRIVE_OFF_BALANCING_STONE_DRIVE_POWER * multiplier)
                }
            }

        }
    }

    private fun alignWithGlyphPit() {
        with(robot) {
            if (cryptoBoxPosition == CryptoBoxPosition.FRONT) {
                driveToDistanceFromObject(
                    rangeSensorDirection(),
                    RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE - 10,
                    ProportionalPowerController(GLYPH_PIT_ALIGNMENT_GAIN),
                    shouldCorrect = false
                )
            }
        }
    }

    private fun alignWithColumn(columnPosition: CryptoBox.ColumnPosition?) {
        robot.driveToDistanceFromObject(
            rangeSensorDirection(),
            wallDistanceForColumn(columnPosition),
            PIDPowerController(
                linearOpMode,
                CRYPTO_BOX_ALIGNMENT_PID_COEFFICIENTS,
                shouldPrintDebug = true
            )
        )
    }

    private fun deliverGlyph() {
        with(robot) {
            driveTrain.linearTimeDrive(
                850,
                StaticPowerController(0.4),
                MecanumDriveTrain.DriveDirection.FORWARD
            )
            glyphGrabber.setState(CodaGlyphGrabbers.GlyphGrabberState.RELEASE)
            driveTrain.linearEncoderDrive(
                GLYPH_DELIVERY_REVERSE_DISTANCE,
                StaticPowerController(0.3),
                1000
            )
        }
    }

    private fun turnToCryptoBox() {
        robot.driveTrain.turnToHeading(
            cryptoBoxHeading,
            ProportionalPowerController(TURNING_GIAN)
        )
    }

    private fun turnToGlyphPit() {
        robot.driveTrain.turnToHeading(glyphPitHeading, ProportionalPowerController(TURNING_GIAN))
    }

    private fun wallDistanceForColumn(position: CryptoBox.ColumnPosition?) =
        with(RelicRecoveryConstants) {
            when (Pair(cryptoBoxPosition, allianceColor)) {

                Pair(CryptoBoxPosition.FRONT, AllianceColor.BLUE) -> {
                    when (position) {
                        CryptoBox.ColumnPosition.LEFT -> LEADING_FRONT_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.CENTER -> CENTER_FRONT_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.RIGHT -> TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                        null -> LEADING_FRONT_CRYPTO_BOX_DISTANCE
                    }
                }

                Pair(CryptoBoxPosition.SIDE, AllianceColor.BLUE) -> {
                    when (position) {
                        CryptoBox.ColumnPosition.LEFT -> LEADING_SIDE_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.CENTER -> CENTER_SIDE_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.RIGHT -> TRAILING_SIDE_CRYPTO_BOX_DISTANCE
                        null -> LEADING_SIDE_CRYPTO_BOX_DISTANCE
                    }
                }

                Pair(CryptoBoxPosition.FRONT, AllianceColor.RED) -> {
                    when (position) {
                        CryptoBox.ColumnPosition.LEFT -> TRAILING_FRONT_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.CENTER -> CENTER_FRONT_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.RIGHT -> LEADING_FRONT_CRYPTO_BOX_DISTANCE
                        null -> LEADING_FRONT_CRYPTO_BOX_DISTANCE
                    }
                }

                Pair(CryptoBoxPosition.SIDE, AllianceColor.RED) -> {
                    when (position) {
                        CryptoBox.ColumnPosition.LEFT -> TRAILING_SIDE_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.CENTER -> CENTER_SIDE_CRYPTO_BOX_DISTANCE
                        CryptoBox.ColumnPosition.RIGHT -> LEADING_SIDE_CRYPTO_BOX_DISTANCE
                        null -> LEADING_SIDE_CRYPTO_BOX_DISTANCE
                    }
                }

                else -> throw NotImplementedError("This should never happen.")
            }
        }

    private fun rangeSensorDirection() =
        if (allianceColor == AllianceColor.BLUE) {
            Coda.RangeSensorDirection.LEFT
        } else {
            Coda.RangeSensorDirection.RIGHT
        }

    private fun jewelConfigurationIsKnown() =
        detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN

    private fun isNotBlueFront() =
        (allianceColor == AllianceColor.BLUE && cryptoBoxPosition == CryptoBoxPosition.FRONT).not()

    companion object {
        private const val SHOULD_GRAB_ADDITIONAL_GLYPHS = false

        private const val DRIVE_ON_BALANCING_STONE_POWER = 0.4
        private const val DRIVE_OFF_BALANCING_STONE_DRIVE_POWER = 0.275
        private const val GLYPH_DELIVERY_DRIVE_POWER = 0.5
        private const val GLYPH_DELIVERY_REVERSE_DRIVE_POWER = 0.6
        private const val GLYPH_PIT_DRIVE_POWER = 1.0
        private const val GLYPH_PIT_REVERSE_DRIVE_POWER = 1.0
        private const val ADDITIONAL_CRYPTO_BOX_APPROACH_DRIVE_POWER = 0.5

        private const val DRIVE_OFF_BALANCING_STONE_DISTANCE = 300
        private const val GLYPH_DELIVERY_REVERSE_DISTANCE = -150
        private const val GLYPH_PIT_DISTANCE = 1500
        private const val GLYPH_PIT_REVERSE_DISTANCE = -1200
        private const val ADDITIONAL_CRYPTO_BOX_APPROACH_DISTANCE = 50

        private const val GLYPH_GRAB_WAIT_TIME = 750L

        private const val STALL_DETECTION_SPEED_THRESHOLD = 2000

        private const val TURNING_GIAN = 0.015
        private const val GLYPH_PIT_ALIGNMENT_GAIN = 0.05

        private val CRYPTO_BOX_ALIGNMENT_PID_COEFFICIENTS =
            PIDCoefficients().apply {
                p = 0.025
                i = 0.015
                d = 0.000
            }
    }

}