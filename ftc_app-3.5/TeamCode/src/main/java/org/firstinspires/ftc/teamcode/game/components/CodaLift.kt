package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.lib.robot.lift.Lift
import kotlin.concurrent.thread

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

    private fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun setPower(power: Double) {
        if(power > 0.0 || !liftIsLowered) {
            super.setPower(Range.clip(power, -0.40, 1.00))
        } else {
            super.setPower(0.0)
        }
    }

    fun drop() {
        if (!liftIsLowered) {
            thread(start = true) {
                setPower(-0.20)
                while (!liftIsLowered) { linearOpMode.idle() }
                setPower(0.0)
                resetEncoder()
            }
        }
    }

}