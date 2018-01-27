package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.lib.powercontroller.PIDPowerController
import org.firstinspires.ftc.teamcode.lib.robot.lift.Lift
import kotlin.concurrent.thread
import kotlin.properties.Delegates

class CodaLift(override val linearOpMode: LinearOpMode) : Lift() {

    override val motor: DcMotor by lazy {
        val motor = hardwareMap.dcMotor.get("lift motor")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor
    }

    private val limitButton: DigitalChannel by lazy {
        val button = hardwareMap.digitalChannel.get("lift limit button")
        button.mode = DigitalChannel.Mode.INPUT
        button
    }

    var shouldHoldLiftPosition = true
    var isBusy = false
        private set

    private val powerController: PIDPowerController by lazy {
        val controller = PIDPowerController(linearOpMode, PID_COEFFICIENTS) {
            motor.currentPosition.toDouble()
        }
        controller.target = targetPosition.value.toDouble()

        controller
    }

    private var targetPosition: LiftPosition by Delegates.observable(
        LiftPosition.BOTTOM
    ) { _, _, new ->
        powerController.target = new.value.toDouble()
    }

    private val liftIsLowered: Boolean
        get() = !limitButton.state

    override fun setPower(power: Double) {
        if (power > 0.0 || !liftIsLowered) {
            super.setPower(Range.clip(power, MINIMUM_POWER, MAXIMUM_POWER))
        } else {
            super.setPower(0.0)
        }
    }

    fun drop() {
        isBusy = true
        if (!liftIsLowered) {
            setPower(-0.20)
            while (!liftIsLowered) {
                linearOpMode.idle()
            }
            setPower(0.0)
            resetEncoder()
        }

        targetPosition = LiftPosition.BOTTOM
        isBusy = false
    }

    enum class LiftPosition(val value: Int) {
        BOTTOM(0),
        FIRST_LEVEL(1200),
        SECOND_LEVEL(1200 * 2),
        THIRD_LEVEL(1200 * 3),
        FORTH_LEVEL(1200 * 4)
    }

    @Synchronized
    fun setPosition(position: LiftPosition) {
        targetPosition = position
    }

    fun elevate() {
        val currentPositionOrdinal = targetPosition.ordinal
        if (currentPositionOrdinal < LiftPosition.values().count() - 1) {
            setPosition(LiftPosition.values()[currentPositionOrdinal + 1])
        }
    }

    fun lower() {
        val currentPositionOrdinal = targetPosition.ordinal
        if (currentPositionOrdinal > 0) {
            setPosition(LiftPosition.values()[currentPositionOrdinal - 1])
        }
    }

    fun startSettingMotorPowers() {
        thread(start = true) {
            while (linearOpMode.opModeIsActive()) {
                if (shouldHoldLiftPosition && !isBusy) {
                    setPower(powerController.output)
                }
            }
        }
    }

    private fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    companion object {
        private const val MINIMUM_POWER = -1.00
        private const val MAXIMUM_POWER = 1.00

        private val PID_COEFFICIENTS = {
            val coefficients = PIDCoefficients()
            coefficients.p = 0.01
            coefficients.i = 0.0
            coefficients.d = 0.0

            coefficients
        }()
    }

}
