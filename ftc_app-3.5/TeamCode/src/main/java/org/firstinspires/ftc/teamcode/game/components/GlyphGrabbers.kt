package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.libraries.robot.attachments.RobotAttachment

class GlyphGrabbers(linearOpMode: LinearOpMode): RobotAttachment(linearOpMode) {

    private val leftGlyphGrabber: Servo = linearOpMode.hardwareMap.servo.get("left grabber")
    private val rightGlyphGrabber: Servo = linearOpMode.hardwareMap.servo.get("right grabber")
    private val glyphDeployer: Servo = linearOpMode.hardwareMap.servo.get("glyph deployer")

    init {
        leftGlyphGrabber.direction = Servo.Direction.FORWARD
        rightGlyphGrabber.direction = Servo.Direction.REVERSE
    }

    enum class GlyphGrabberState(val armPosition: Double, val deployerPosition: GlyphDeployerPosition) {
        OPEN(0.50, GlyphDeployerPosition.RETRACTED),
        SMALL_OPEN(0.66, GlyphDeployerPosition.RETRACTED),
        RELEASE(0.75, GlyphDeployerPosition.EXTENDED),
        CLOSED(0.90, GlyphDeployerPosition.EXTENDED)
    }

    enum class GlyphDeployerPosition(val value: Double) {
        EXTENDED(0.25),
        RETRACTED(0.90),
        UP(0.01)
    }

    private fun setGlyphArmsPosition(position: Double) {
        leftGlyphGrabber.position = position
        rightGlyphGrabber.position = position
    }

    private fun setGlyphDeployerPosition(position: GlyphDeployerPosition) {
        glyphDeployer.position = position.value
    }

    fun setGlyphGrabberState(state: GlyphGrabberState, delay: Long = 0) {
        setGlyphArmsPosition(state.armPosition)
        setGlyphDeployerPosition(state.deployerPosition)
        linearOpMode.sleep(delay)
    }

}