package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.lib.robot.lift.LimitedRobotLift

class Lift(linearOpMode: LinearOpMode,
           motorName: String = "winch motor",
           direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD,
           limitDeviceName: String = "limit switch") :
        LimitedRobotLift(linearOpMode, motorName, direction, limitDeviceName) {

    /**
     * A position to set the lift to.
     */
    enum class LiftPosition(val encoderPosition: Int) {
        FOURTH_LEVEL(2950),
        THIRD_LEVEL(2000),
        SECOND_LEVEL(1150),
        FIRST_LEVEL(400),
        BOTTOM_LEVEL(0)
    }

    init {
        positionCorrectionProportionalCoefficient = 0.002500
        positionCorrectionIntegralCoefficient     = 0.000500
        positionCorrectionDerivativeCoefficient   = 0.000035
    }

    fun setPosition(targetPosition: LiftPosition, power: Double = 0.75) {
        setPosition(targetPosition.encoderPosition, power)
    }

    override fun drop(power: Double) {
        setPosition(LiftPosition.BOTTOM_LEVEL)
    }
}