package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class CodaGlyphGrabber(linearOpMode: LinearOpMode): RobotAttachment(linearOpMode) {

    var currentState: GlyphGrabberState = GlyphGrabberState.OPEN
        private set

    private val leftGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("left grabber")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    private val rightGlyphGrabber: Servo by lazy {
        val servo = hardwareMap.servo.get("right grabber")
        servo.direction = Servo.Direction.FORWARD
        servo
    }

    private val glyphDeployer: Servo by lazy {
        val servo = hardwareMap.servo.get("glyph deployer")
        servo.direction = Servo.Direction.REVERSE
        servo
    }

    enum class GlyphArmPosition(val value: Double) {
        OPEN(0.0),
        SMALL_OPEN(0.15),
        RELEASE(0.25),
        CLOSED(0.35)
    }

    enum class GlyphDeployerPosition(val value: Double) {
        RETRACTED(0.075),
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
        leftGlyphGrabber.position = position.value
        rightGlyphGrabber.position = position.value
    }

    private fun setDeployerPosition(position: GlyphDeployerPosition) {
        glyphDeployer.position = position.value
    }

    fun setState(state: GlyphGrabberState, delay: Long = 0) {
        currentState = state
        setDeployerPosition(state.deployerPosition)
        setArmsPosition(state.armPosition)
        linearOpMode.sleep(delay)
    }

}