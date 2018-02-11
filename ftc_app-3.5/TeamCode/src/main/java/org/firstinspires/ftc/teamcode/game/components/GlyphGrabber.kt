package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class CodaGlyphGrabber(linearOpMode: LinearOpMode): RobotAttachment(linearOpMode) {

    var currentState: GlyphGrabberState = GlyphGrabberState.OPEN
        private set

    private val topLeftGlyphGrabber by lazy {
        val servo = hardwareMap.servo.get("top left grabber")
        servo.direction = Servo.Direction.FORWARD
        servo
    }

    private val topRightGlyphGrabber by lazy {
        val servo = hardwareMap.servo.get("top right grabber")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    private val bottomLeftGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("bottom left grabber")
        servo.direction = Servo.Direction.FORWARD
        servo
    }

    private val bottomRightGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("bottom right grabber")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    private val glyphDeployer: Servo by lazy {
        val servo = hardwareMap.servo.get("glyph deployer")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    enum class GlyphArmPosition(val value: Double) {
        OPEN(0.0),
        SMALL_OPEN(0.30),
        RELEASE(0.45),
        CLOSED(0.55)
    }

    enum class GlyphDeployerPosition(val value: Double) {
        RETRACTED(0.375),
        EXTENDED(0.65)
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

    private fun setArmsPosition(position: GlyphArmPosition) {
        topLeftGlyphGrabber.position = position.value
        topRightGlyphGrabber.position = position.value
        bottomLeftGlyphGrabber.position = position.value
        bottomRightGlyphGrabber.position = position.value
    }

    private fun setDeployerPosition(position: GlyphDeployerPosition) {
        glyphDeployer.position = position.value
    }

    fun setState(state: GlyphGrabberState, delay: Long = 0) {
        if (!linearOpMode.isStopRequested) {
            currentState = state
            setDeployerPosition(state.deployerPosition)
            setArmsPosition(state.armPosition)
            linearOpMode.sleep(delay)
        }
    }

}