package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.lib.robot.lift.Lift

class CodaLift(override val linearOpMode: LinearOpMode) : Lift() {

    override val motor: DcMotor by lazy {
        val motor = hardwareMap.dcMotor.get("lift motor")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor
    }

    private val limitButton: DigitalChannel by lazy {
        val button = hardwareMap.digitalChannel.get("lift limit button")
        button.mode = DigitalChannel.Mode.INPUT
        button
    }

    private val liftIsLowered: Boolean
        get() = !limitButton.state

    enum class LiftPosition(val value: Int) {
        BOTTOM(0),
        FIRST_LEVEL(250),
        SECOND_LEVEL(0),
        THIRD_LEVEL(0),
        FORTH_LEVEL(0)
    }

    private var currentPosition: LiftPosition = LiftPosition.BOTTOM

    private fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun setPower(power: Double) {
        if(power > 0.0 || !liftIsLowered) {
            super.setPower(Range.clip(power, MINIMUM_LIFT_POWER, MAXIMUM_LIFT_POWER))
        } else {
            super.setPower(0.0)
        }
    }

    fun drop() {
        if (!liftIsLowered) {
            setPower(-0.20)
            while (!liftIsLowered) { linearOpMode.idle() }
            setPower(0.0)
            resetEncoder()
        }
    }

    fun setPosition(position: LiftPosition) {
        currentPosition = position
        if (position.value > motor.currentPosition) {
            setPower(MAXIMUM_LIFT_POWER)
            while (position.value > motor.currentPosition) { linearOpMode.idle() }
        } else if (position.value < motor.currentPosition) {
            setPower(MINIMUM_LIFT_POWER)
            while (position.value < motor.currentPosition) { linearOpMode.idle() }
        }
        setPower(0.0)
    }

    companion object {
        private const val MINIMUM_LIFT_POWER = -0.40
        private const val MAXIMUM_LIFT_POWER = 1.00
    }

}