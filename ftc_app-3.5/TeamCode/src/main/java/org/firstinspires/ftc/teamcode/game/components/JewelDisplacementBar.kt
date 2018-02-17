package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class CodaJewelDisplacementBar(
    linearOpMode: LinearOpMode,
    name: String = "jewel stick"
) : RobotAttachment(linearOpMode) {

    private val servo by lazy {
        val servo = hardwareMap.servo.get(name)
        servo.scaleRange(SERVO_MIN_POSITION, SERVO_MAX_POSITION)
        servo
    }

    var currentPosition = Position.UP
        private set

    enum class Position(val value: Double) {
        UP(0.0),
        DOWN(1.0)
    }

    fun setPosition(position: Position, delay: Long = 0) {
        currentPosition = position
        setPosition(position.value)
        linearOpMode.sleep(delay)
    }

    private fun setPosition(position: Double) {
        servo.position = position
    }

    companion object {
        private const val SERVO_MIN_POSITION = 0.0
        private const val SERVO_MAX_POSITION = 0.885
    }

}