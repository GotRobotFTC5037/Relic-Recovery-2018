package org.firstinspires.ftc.teamcode.game.actions

import OpModeController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.game.RelicRecoveryConstants
import org.firstinspires.ftc.teamcode.game.components.CodaGlyphGrabber
import org.firstinspires.ftc.teamcode.game.components.CodaJewelDisplacementBar
import org.firstinspires.ftc.teamcode.game.components.CodaLift
import org.firstinspires.ftc.teamcode.game.elements.CryptoBox
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.game.vision.JewelConfigurationDetector
import org.firstinspires.ftc.teamcode.game.vision.PictographIdentifier
import org.firstinspires.ftc.teamcode.lib.powercontroller.PIDPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.ProportionalPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.StaticPowerController
import org.firstinspires.ftc.teamcode.lib.robot.drivetrain.Heading
import kotlin.concurrent.thread

/**
 * The class used to run all action that are programed.
 */
class CodaAutonomousActions(

    /**
     * The linear op mode that the robot is running on.
     */
    val linearOpMode: LinearOpMode,

    /**
     * The robot to be used to perform actions.
     */
    val robot: Coda = Coda(linearOpMode)

) {

    /**
     * The alliance color of the robot to be used to perform actions.
     */
    var allianceColor = AllianceColor.UNDETERMINED

    /**
     * An enum class for the current alliance color.
     */
    enum class AllianceColor {

        /**
         * Used when the alliance color is red.
         */
        RED,

        /**
         * Used when the alliance color is blue.
         */
        BLUE,

        /**
         * Used when the alliance color has not been determined.
         */
        UNDETERMINED
    }

    /**
     * The position of the target crypto box.
     */
    var cryptoBoxPosition = CryptoBoxPosition.UNDETERMINED

    /**
     * An enum class for the target crypto box position.
     */
    enum class CryptoBoxPosition {

        /**
         * Used when the crypto box is facing the audience.
         */
        FRONT,

        /**
         * Used when the crypto box is facing the alliance team members.
         */
        SIDE,

        /**
         * Used when the crypto box position has not been determined.
         */
        UNDETERMINED
    }

    /**
     * The heading of the target crypto box while the robot is at the glyph pit.
     */
    var cryptoBoxHeading: Heading = 0.0

    /**
     * The heading of the glyph pit while the robot is at the crypto box.
     */
    var glyphPitHeading: Heading = 0.0

    private val jewelConfigurationDetector by lazy { JewelConfigurationDetector(linearOpMode) }
    private var detectedJewelConfiguration = JewelConfigurationDetector.JewelConfiguration.UNKNOWN

    private val pictographIdentifier by lazy { PictographIdentifier(linearOpMode) }
    private var detectedPictographColumn: CryptoBox.ColumnPosition? = null

    private fun rangeSensorDirection() =
        if (allianceColor == AllianceColor.BLUE) {
            Coda.RangeSensorDirection.LEFT
        } else {
            Coda.RangeSensorDirection.RIGHT
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

                else -> {
                    // This should never happen
                    0.0
                }

            }
        }

    /**
     * Performs all of the actions done inside of our autonomous opmode.
     */
    @Throws(InterruptedException::class)
    fun performForAutonomousOpMode() {

        with(robot) {

            // Setup the robot.
            setup()

            // Setup the crypto box.
            val cryptoBox = CryptoBox()

            // Queue TeleOp.
            OpModeController.queueOpMode(linearOpMode, "TeleOp")

            // Setup the jewel configuration detector and pictograph identifier.
            val context = linearOpMode.hardwareMap.appContext
            val viewDisplay = CameraViewDisplay.getInstance()
            jewelConfigurationDetector.init(context, viewDisplay)

            // Wait for start.
            linearOpMode.waitForStart()

            // Enable the jewel configuration detector.
            jewelConfigurationDetector.enable()

            // Grab the glyph.
            val glyphGrabbingThread = thread(start = true) {
                glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.CLOSED)
                linearOpMode.sleep(750)
                lift.position = CodaLift.LiftPosition.FIRST_LEVEL
            }

            // Detect the jewel configuration.
            detectedJewelConfiguration = jewelConfigurationDetector.waitForJewelIdentification()

            // Move the jewel displacement bar if applicable.
            if (detectedJewelConfiguration != JewelConfigurationDetector.JewelConfiguration.UNKNOWN) {
                jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.DOWN)
            }

            // Disable the jewel detector and active the pictograph identifier.
            jewelConfigurationDetector.disable()
            pictographIdentifier.activate()

            // Detect the pictograph.
            detectedPictographColumn = pictographIdentifier.waitForPictographIdentification()
            pictographIdentifier.deactivate()

            // Wait for the grabber to grab the glyph.
            glyphGrabbingThread.join()

            // Displace the correct jewel.
            displaceIdentifiedJewel()

            // Turn to the crypto box
            if (!(allianceColor == AllianceColor.BLUE && cryptoBoxPosition == CryptoBoxPosition.FRONT)) {
                turnToCryptoBox()
            }

            // Align with the correct crypto box column and deliver glyph.
            alignWithColumn(detectedPictographColumn)
            lift.drop()
            deliverGlyph()

            // Add the glyph to virtual crypto box
            cryptoBox.addGlyphToColumn(
                glyphGrabber.grabbedGlyphs.bottomGlyph!!,
                detectedPictographColumn!!
            )

            // Shift the robot to a better position for glyph grabbing.
            if (cryptoBoxPosition == CryptoBoxPosition.FRONT) {
                driveToDistanceFromObject(
                    rangeSensorDirection(),
                    RelicRecoveryConstants.TRAILING_FRONT_CRYPTO_BOX_DISTANCE,
                    ProportionalPowerController(0.05),
                    shouldCorrect = false
                )
            } else if (cryptoBoxPosition == CryptoBoxPosition.SIDE) {
                driveToDistanceFromObject(
                    rangeSensorDirection(),
                    RelicRecoveryConstants.CENTER_SIDE_CRYPTO_BOX_DISTANCE,
                    ProportionalPowerController(0.05),
                    shouldCorrect = false
                )
            }

            // Continually grab more glyphs until the OpMode ends.
            while (linearOpMode.isStopRequested.not() && cryptoBox.isFull.not()) {

                // Prepare to grab a glyph.
                turnToGlyphPit()
                lift.drop()
                glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.SMALL_OPEN)

                // Drive to the glyph pit and grab a glyph.
                driveTrain.linearEncoderDrive(850, StaticPowerController(1.0))
                glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.CLOSED)
                linearOpMode.sleep(500)

                // Determine where to put the glyph in the crypto box.
                val grabbedGlyphs = glyphGrabber.grabbedGlyphs
                val position = cryptoBox.positionForGlyphs(grabbedGlyphs)!!

                // Deliver glyph.
                lift.moveToRow(position.row)
                driveTrain.linearEncoderDrive(-700, StaticPowerController(1.0))
                turnToCryptoBox()
                driveTrain.linearEncoderDrive(150, StaticPowerController(0.4))
                alignWithColumn(position.column)
                deliverGlyph()

                // Add the glyph to the virtual crypto box.
                glyphGrabber.grabbedGlyphs.bottomGlyph?.let {
                    cryptoBox.addGlyphToColumn(it, position.column)
                }

            }

        }

    }

    private fun displaceIdentifiedJewel() {
        with(robot) {
            val multiplier = if (allianceColor == AllianceColor.BLUE) 1 else -1
            when (detectedJewelConfiguration) {
                JewelConfigurationDetector.JewelConfiguration.RED_BLUE -> {
                    driveTrain.linearEncoderDrive(
                        300 * multiplier * -1,
                        StaticPowerController(0.175)
                    )
                    jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                    driveOnBalancingStone(StaticPowerController(0.40))
                    driveOffBalancingStone(0.175 * multiplier)
                }

                JewelConfigurationDetector.JewelConfiguration.BLUE_RED -> {
                    driveOffBalancingStone(0.175 * multiplier)
                    jewelDisplacementBar.setPosition(CodaJewelDisplacementBar.Position.UP)
                }

                JewelConfigurationDetector.JewelConfiguration.UNKNOWN -> {
                    driveOffBalancingStone(0.175 * multiplier)
                }
            }

        }
    }

    private fun turnToCryptoBox() {
        robot.driveTrain.turnToHeading(
            cryptoBoxHeading,
            CodaAutonomousActions.TURN_POWER_CONTROLLER
        )
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
            driveTrain.linearStallDetectionDrive(speedThreshold = 300, power = 0.5, timeout = 1000)
            glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.RELEASE)
            driveTrain.linearEncoderDrive(-200, StaticPowerController(0.25))
            glyphGrabber.setState(CodaGlyphGrabber.GlyphGrabberState.SMALL_OPEN)
        }
    }

    private fun turnToGlyphPit() {
        robot.driveTrain.turnToHeading(glyphPitHeading, CodaAutonomousActions.TURN_POWER_CONTROLLER)
    }

    companion object {

        /**
         * The power controller used to control the power the robot drives at while it is turning.
         */
        val TURN_POWER_CONTROLLER = ProportionalPowerController(0.02)

        /**
         * The power controller used to control the power the robot strafes at while delivering
         * glyphs.
         */
        val CRYPTO_BOX_ALIGNMENT_PID_COEFFICIENTS =
            PIDCoefficients().apply {
                p = 0.025
                i = 0.015
                d = 0.000
            }

    }

}