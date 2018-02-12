package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class CodaGlyphGrabber(linearOpMode: LinearOpMode): RobotAttachment(linearOpMode) {

    private val topLeftGlyphGrabber by lazy {
        val servo = hardwareMap.servo.get("top left grabber")
        servo.direction = Servo.Direction.FORWARD
        servo.scaleRange(GRABBER_SERVO_MIN_POSITION, GRABBER_SERVO_MAX_POSITION)
        servo
    }

    private val topRightGlyphGrabber by lazy {
        val servo = hardwareMap.servo.get("top right grabber")
        servo.direction = Servo.Direction.REVERSE
        servo.scaleRange(GRABBER_SERVO_MIN_POSITION, GRABBER_SERVO_MAX_POSITION)
        servo
    }

    private val bottomLeftGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("bottom left grabber")
        servo.direction = Servo.Direction.FORWARD
        servo.scaleRange(GRABBER_SERVO_MIN_POSITION, GRABBER_SERVO_MAX_POSITION)
        servo
    }

    private val bottomRightGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("bottom right grabber")
        servo.direction = Servo.Direction.REVERSE
        servo.scaleRange(GRABBER_SERVO_MIN_POSITION, GRABBER_SERVO_MAX_POSITION)
        servo
    }

    private val glyphDeployer: Servo by lazy {
        val servo = hardwareMap.servo.get("glyph deployer")
        servo.direction = Servo.Direction.REVERSE
        servo.scaleRange(DEPLOYER_SERVO_MIN_POSITION, DEPLOYER_SERVO_MAX_POSITION)
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

    enum class GlyphArmPosition(val value: Double) {
        OPEN(0.0),
        SMALL_OPEN(0.50),
        RELEASE(0.75),
        CLOSED(1.0)
    }

    private fun setArmsPosition(position: GlyphArmPosition) {
        topLeftGlyphGrabber.position = position.value
        topRightGlyphGrabber.position = position.value
        bottomLeftGlyphGrabber.position = position.value
        bottomRightGlyphGrabber.position = position.value
    }

    enum class GlyphDeployerPosition(val value: Double) {
        RETRACTED(0.0),
        EXTENDED(1.0)
    }

    private fun setDeployerPosition(position: GlyphDeployerPosition) {
        glyphDeployer.position = position.value
    }

    companion object {
        private const val GRABBER_SERVO_MIN_POSITION  = 0.0
        private const val GRABBER_SERVO_MAX_POSITION  = 0.5

        private const val DEPLOYER_SERVO_MIN_POSITION = 0.375
        private const val DEPLOYER_SERVO_MAX_POSITION = 0.650
    }

}