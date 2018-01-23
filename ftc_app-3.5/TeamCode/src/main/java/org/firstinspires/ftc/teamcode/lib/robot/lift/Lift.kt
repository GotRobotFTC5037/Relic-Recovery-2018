package org.firstinspires.ftc.teamcode.lib.robot.lift

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.lib.robot.Component

abstract class Lift : Component {

    abstract val motor: DcMotor

    open fun setPower(power: Double) {
        motor.power = power
    }

}