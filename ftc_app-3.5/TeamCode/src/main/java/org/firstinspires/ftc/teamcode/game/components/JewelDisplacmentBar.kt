package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.robot.attachment.RobotAttachment

class JewelDisplacmentBar(linearOpMode: LinearOpMode, name: String = "jewel stick"): RobotAttachment(linearOpMode) {

    enum class Position(val value: Double) {
        UP(0.0),
        DOWN(0.885)
    }

    private val servo = linearOpMode.hardwareMap.servo.get(name)

    private fun setPosition(position: Double) {
        servo.position = position
    }

    fun setPosition(position: Position, delay: Long = 0) {
        setPosition(position.value)
        linearOpMode.sleep(delay)
    }
}