package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.game.elements.Glyph
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

/**
 * The grabbers on the Coda used to grab, identify and deploy glyphs.
 */
class CodaGlyphGrabber(linearOpMode: LinearOpMode) : RobotAttachment(linearOpMode) {

    val topLeftGlyphGrabber: Servo by lazy {
        hardwareMap.servo.get("top left grabber")
            .apply { direction = Servo.Direction.REVERSE }
    }

    val topRightGlyphGrabber: Servo by lazy {
        hardwareMap.servo.get("top right grabber")
            .apply { direction = Servo.Direction.FORWARD }
    }

    val bottomLeftGlyphGrabber: Servo by lazy {
        hardwareMap.servo.get("bottom left grabber")
            .apply { direction = Servo.Direction.REVERSE }
    }

    val bottomRightGlyphGrabber: Servo by lazy {
        hardwareMap.servo.get("bottom right grabber")
            .apply { direction = Servo.Direction.FORWARD }
    }

    private val glyphDeployer by lazy {
        hardwareMap.servo.get("glyph deployer")
            .apply { direction = Servo.Direction.FORWARD }
    }

    private val topColorSensor: ColorSensor by lazy {
        hardwareMap.colorSensor.get("top glyph color sensor")
    }

    private val bottomColorSensor: ColorSensor by lazy {
        hardwareMap.colorSensor.get("bottom glyph color sensor")
    }

    data class GrabbedGlyphs(
        val topGlyph: Glyph?,
        val bottomGlyph: Glyph?
    )

    val grabbedGlyphs: GrabbedGlyphs
        get() = GrabbedGlyphs(
            topGlyph = glyphDetectedByColorSensor(topColorSensor),
            bottomGlyph = glyphDetectedByColorSensor(bottomColorSensor)
        )

    private fun glyphDetectedByColorSensor(sensor: ColorSensor): Glyph? {
        // TODO: Return the actual glyph detected by the color sensor.
        return Glyph(Glyph.Color.UNKNOWN)
    }

    enum class GlyphGrabberState(
        val armPosition: GlyphArmPosition,
        val deployerPosition: GlyphDeployerPosition
    ) {
        OPEN(GlyphArmPosition.OPEN, GlyphDeployerPosition.RETRACTED),
        CLOSED(GlyphArmPosition.CLOSED, GlyphDeployerPosition.RETRACTED),
        SMALL_OPEN(GlyphArmPosition.SMALL_OPEN, GlyphDeployerPosition.RETRACTED),
        RELEASE(GlyphArmPosition.RELEASE, GlyphDeployerPosition.EXTENDED),
    }

    var currentState: GlyphGrabberState = GlyphGrabberState.OPEN
        private set

    fun setState(state: GlyphGrabberState, delay: Long = 0) {
        if (!linearOpMode.isStopRequested) {
            currentState = state
            setDeployerPosition(state.deployerPosition)
            setArmsPosition(state.armPosition)
            linearOpMode.sleep(delay)
        }
    }

    enum class GlyphArmPosition(
        val bottomGrabberPosition: Double,
        val topGrabberPosition: Double
    ) {
        OPEN(BOTTOM_GRABBER_OPEN_POSITION, TOP_GRABBER_OPEN_POSITION),
        SMALL_OPEN(BOTTOM_GRABBER_SMALL_OPEN_POSITION, TOP_GRABBER_SMALL_OPEN_POSITION),
        RELEASE(BOTTOM_GRABBER_RELEASE_POSITION, TOP_GRABBER_RELEASE_POSITION),
        CLOSED(BOTTOM_GRABBER_CLOSED_POSITION, TOP_GRABBER_CLOSED_POSITION)
    }

    private fun setArmsPosition(position: GlyphArmPosition) {
        topLeftGlyphGrabber.position = position.topGrabberPosition
        topRightGlyphGrabber.position = position.topGrabberPosition
        bottomLeftGlyphGrabber.position = position.bottomGrabberPosition
        bottomRightGlyphGrabber.position = position.bottomGrabberPosition
    }

    enum class GlyphDeployerPosition(
        val value: Double
    ) {
        RETRACTED(DEPLOYER_SERVO_MIN_POSITION),
        EXTENDED(DEPLOYER_SERVO_MAX_POSITION)
    }

    private fun setDeployerPosition(position: GlyphDeployerPosition) {
        glyphDeployer.position = position.value
    }

    companion object {
        private const val BOTTOM_GRABBER_OPEN_POSITION = 0.0
        private const val BOTTOM_GRABBER_SMALL_OPEN_POSITION = 0.1
        private const val BOTTOM_GRABBER_RELEASE_POSITION = 0.25
        const val BOTTOM_GRABBER_CLOSED_POSITION = 0.5

        private const val TOP_GRABBER_OPEN_POSITION = 0.0
        private const val TOP_GRABBER_SMALL_OPEN_POSITION = 0.1
        private const val TOP_GRABBER_RELEASE_POSITION = 0.35
        private const val TOP_GRABBER_CLOSED_POSITION = 0.5

        private const val DEPLOYER_SERVO_MIN_POSITION = 0.5
        private const val DEPLOYER_SERVO_MAX_POSITION = 0.1

        private const val GLYPH_ALPHA_THRESHOLD = 0.0
    }

}