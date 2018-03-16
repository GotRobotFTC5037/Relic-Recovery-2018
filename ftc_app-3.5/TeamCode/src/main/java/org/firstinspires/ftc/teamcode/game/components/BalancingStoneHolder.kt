package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.robot.Component

/** The stone holder that prevents the robot from over shooting the balancing stone. */
class CodaBalancingStoneHolder(override val linearOpMode: LinearOpMode) : Component {

    private val servo by lazy {
        val servo = hardwareMap.servo.get("balancing stone holder")
        servo.direction = Servo.Direction.REVERSE
        servo

    }

    enum class BalancingStoneHolderState(val value: Double) {
        UP(UP_POSITION),
        DOWN(DOWN_POSITION)
    }

    fun setState(state: BalancingStoneHolderState) {
        servo.position = state.value
    }

    companion object {
        private const val UP_POSITION = 1.0
        private const val DOWN_POSITION = 0.2
    }

}