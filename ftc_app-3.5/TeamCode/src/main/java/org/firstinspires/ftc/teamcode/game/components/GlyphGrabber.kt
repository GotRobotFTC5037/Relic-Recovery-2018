package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class CodaGlyphGrabber(linearOpMode: LinearOpMode): RobotAttachment(linearOpMode) {

    val topLeftGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("top left grabber")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    val topRightGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("top right grabber")
        servo.direction = Servo.Direction.FORWARD
        servo
    }

    val bottomLeftGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("bottom left grabber")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    val bottomRightGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("bottom right grabber")
        servo.direction = Servo.Direction.FORWARD
        servo
    }

    private val glyphDeployer by lazy {
        val servo = hardwareMap.servo.get("glyph deployer")
        servo.direction = Servo.Direction.FORWARD
        servo
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

    enum class GlyphArmPosition(val bottomGrabberPosition: Double, val topGrabberPosition: Double) {
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

    enum class GlyphDeployerPosition(val value: Double) {
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
    }

}