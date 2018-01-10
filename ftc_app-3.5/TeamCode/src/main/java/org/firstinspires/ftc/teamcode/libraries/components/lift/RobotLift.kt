package org.firstinspires.ftc.teamcode.libraries.components.lift

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.concurrent.thread
import kotlin.math.abs


/**
 * A general class for lifts that are run by a single motor and have an encoder attached.
 *
 * @author FTC Team 5037 gotrobot?
 */
open class RobotLift(private val linearOpMode: LinearOpMode, motorName: String, direction: DcMotorSimple.Direction) {

    val motor: DcMotor = linearOpMode.hardwareMap.dcMotor.get(motorName)
    private var shouldHoldPosition = true
    private var targetPosition = 0
    private var isManuallyMovingLift = false
    var positionCorrectionCoefficient = 0.0

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = direction
        //beginHoldingPosition()
    }

    open fun liftIsLowered() = motor.currentPosition <= 0

    private fun getPositionCorrectionPower(): Double = (targetPosition - motor.currentPosition) * positionCorrectionCoefficient

    open fun manuallyMove(power: Double) {
        if (!linearOpMode.isStopRequested) {
            motor.power = power
            if (abs(power) == 0.0) {
                targetPosition = motor.currentPosition
                shouldHoldPosition = true
                isManuallyMovingLift = false
            } else {
                shouldHoldPosition = false
                isManuallyMovingLift = true
            }
        }
    }

    @Synchronized
    open fun setPosition(targetPosition: Int, power: Double = 0.5) {
        if (!linearOpMode.isStopRequested) {
            shouldHoldPosition = false
            this.targetPosition = targetPosition

            when {
                targetPosition > motor.currentPosition -> {
                    motor.power = abs(power)
                    while (targetPosition > motor.currentPosition && !isManuallyMovingLift && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    motor.power = 0.0
                }

                targetPosition < motor.currentPosition -> {
                    motor.power = -abs(power)
                    while (targetPosition < motor.currentPosition && !isManuallyMovingLift && !linearOpMode.isStopRequested) {
                        linearOpMode.sleep(10)
                    }
                    motor.power = 0.0
                }
            }

            shouldHoldPosition = true
        }
    }

    open fun drop(power: Double = 0.5) {
        setPosition(0, power)
    }

    private fun beginHoldingPosition() {
        if (!linearOpMode.isStopRequested) {
            thread(start = true) {
                while(linearOpMode.opModeIsActive()) {
                    if(shouldHoldPosition) {
                        motor.power = getPositionCorrectionPower()
                    }
                }
            }
        }
    }

}