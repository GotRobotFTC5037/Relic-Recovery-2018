package org.firstinspires.ftc.teamcode.libraries.components.lift

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel

open class LimitedRobotLift(linearOpMode: LinearOpMode, motorName: String, direction: DcMotorSimple.Direction, limitDeviceName: String): RobotLift(linearOpMode, motorName, direction) {

    val limitDevice: DigitalChannel = linearOpMode.hardwareMap.digitalChannel.get(limitDeviceName)

    init {
        limitDevice.mode = DigitalChannel.Mode.INPUT
    }

    override fun manuallyMove(power: Double) {

        if (motor.mode != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            if (power > 0.0 || !liftIsLowered()) {
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }

        super.manuallyMove(power)

        if (liftIsLowered() && motor.mode != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            motor.power = 0.0
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        }

    }

    override fun liftIsLowered() = !limitDevice.state

}