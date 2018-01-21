package org.firstinspires.ftc.teamcode.libraries.robot.lift

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.libraries.robot.Component
import kotlin.concurrent.thread
import kotlin.math.abs


/**
 * A general class for lifts that are run by a single motor and have an encoder attached.
 *
 * @author FTC Team 5037 gotrobot?
 */
open class RobotLift(override val linearOpMode: LinearOpMode, motorName: String, direction: DcMotorSimple.Direction): Component {

    private val motor: DcMotor = linearOpMode.hardwareMap.dcMotor.get(motorName)

    private var previousError = 0
    private var integral = 0.0
    private var deltaTime = ElapsedTime(ElapsedTime.Resolution.MILLISECONDS)

    private var shouldHoldPosition = true
    private var targetPosition = 0
    private var isManuallyMovingLift = false

    private var isBusy = false
    var positionCorrectionProportionalCoefficient = 0.0
    var positionCorrectionIntegralCoefficient = 0.0
    var positionCorrectionDerivativeCoefficient = 0.0

    init {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = direction
    }

    open fun liftIsLowered() = motor.currentPosition <= 0

    private fun getPositionCorrectionPower(): Double {
        val dt = deltaTime.milliseconds() / 1000

        val error = (targetPosition - motor.currentPosition)
        integral += error * dt
        val derivative = (error - previousError) / dt

        val proportionalOutput = error * positionCorrectionProportionalCoefficient
        val integralOutput = integral * positionCorrectionIntegralCoefficient
        val derivativeOutput = derivative * positionCorrectionDerivativeCoefficient
        val output = proportionalOutput + integralOutput + derivativeOutput

        previousError = error
        deltaTime.reset()

        return output
    }

    open fun manuallyMove(power: Double) {
        if (!linearOpMode.isStopRequested) {
            if (!isBusy) {
                if ((power > 0.0) || (power < 0.0 && !liftIsLowered())) {
                    shouldHoldPosition = false
                    isManuallyMovingLift = true
                    motor.power = power
                } else if (isManuallyMovingLift) {
                    targetPosition = motor.currentPosition
                    shouldHoldPosition = true
                    isManuallyMovingLift = false
                }
            }
        }
    }

    open fun setPosition(targetPosition: Int, power: Double = 0.5) {
        if (!linearOpMode.isStopRequested) {
            isBusy = true
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
            isBusy = false
        }
    }

    open fun drop(power: Double = 0.5) {
        setPosition(0, power)
    }

    fun beginHoldingPosition() {
        if (!linearOpMode.isStopRequested) {
            thread(start = true) {
                deltaTime.reset()
                while(linearOpMode.opModeIsActive()) {
                    if(shouldHoldPosition) {
                        motor.power = getPositionCorrectionPower()
                        linearOpMode.sleep(10)
                    }
                }
            }
        }

    }

}