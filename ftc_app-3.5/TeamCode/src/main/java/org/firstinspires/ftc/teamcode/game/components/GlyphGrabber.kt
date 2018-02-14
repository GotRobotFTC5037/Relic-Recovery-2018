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
        servo.scaleRange(DEPLOYER_SERVO_MIN_POSITION, DEPLOYER_SERVO_MAX_POSITION)
        servo.direction = Servo.Direction.REVERSE
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
        OPEN(GRABBER_SERVO_MIN_POSITION),
        SMALL_OPEN(0.10),
        RELEASE(0.30),
        CLOSED(GRABBER_SERVO_MAX_POSITION)
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
        private const val GRABBER_SERVO_MAX_POSITION  = 0.6

        private const val DEPLOYER_SERVO_MIN_POSITION = 0.375
        private const val DEPLOYER_SERVO_MAX_POSITION = 0.650
    }

}