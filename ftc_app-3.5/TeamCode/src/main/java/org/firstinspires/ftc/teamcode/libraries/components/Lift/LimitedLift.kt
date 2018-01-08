package org.firstinspires.ftc.teamcode.libraries.components.Lift

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel

open class LimitedLift(linearOpMode: LinearOpMode, motorName: String, direction: DcMotorSimple.Direction, binaryLimitDeviceName: String): Lift(linearOpMode, motorName, direction) {

    private val binaryLimitDevice: DigitalChannel = linearOpMode.hardwareMap.digitalChannel.get(binaryLimitDeviceName)

    init {
        binaryLimitDevice.mode = DigitalChannel.Mode.INPUT
    }

    override fun manuallyMove(power: Double) {

        if (motor.mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            if (power > 0.0 || !binaryLimitDevice.state) {
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }

        super.manuallyMove(power)

        if (binaryLimitDevice.state && motor.mode != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motor.power = 0.0
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

    }

    override fun liftIsLowered() = binaryLimitDevice.state

}