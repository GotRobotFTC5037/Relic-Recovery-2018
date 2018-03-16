package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.game.elements.CryptoBox
import org.firstinspires.ftc.teamcode.lib.powercontroller.PIDPowerController
import org.firstinspires.ftc.teamcode.lib.robot.lift.Lift
import kotlin.concurrent.thread

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
    private var isBusy = false

    private val powerController: PIDPowerController by lazy {
        val controller = PIDPowerController(linearOpMode, PID_COEFFICIENTS)
        controller.errorValueHandler = {
            position.value - motor.currentPosition.toDouble()
        }
        controller
    }

    var position: LiftPosition = LiftPosition.BOTTOM

    private val isLowered: Boolean
        get() = !limitButton.state

    override fun setPower(power: Double) {
        if (power > 0.0 || !isLowered) {
            super.setPower(Range.clip(power, MINIMUM_POWER, MAXIMUM_POWER))
        } else {
            if (isLowered) {
                resetEncoder()
            }
            super.setPower(0.0)
        }
    }

    fun moveToRow(row: CryptoBox.RowPosition) {
        position = when (row) {
            CryptoBox.RowPosition.FIRST -> LiftPosition.BOTTOM
            CryptoBox.RowPosition.SECOND -> LiftPosition.FIRST_LEVEL
            CryptoBox.RowPosition.THIRD -> LiftPosition.SECOND_LEVEL
            CryptoBox.RowPosition.FOURTH -> LiftPosition.THIRD_LEVEL
        }
    }

    enum class LiftPosition(var value: Int) {
        BOTTOM(0),
        FIRST_LEVEL(1750),
        SECOND_LEVEL(3100),
        THIRD_LEVEL(4450)
    }

    fun elevate() {
        val currentPositionOrdinal = position.ordinal
        if (currentPositionOrdinal < LiftPosition.values().count() - 1) {
            position = LiftPosition.values()[currentPositionOrdinal + 1]
        }
    }

    fun lower() {
        val currentPositionOrdinal = position.ordinal
        if (currentPositionOrdinal > 0) {
            position = LiftPosition.values()[currentPositionOrdinal - 1]
        }
    }

    fun drop(timeout: Long = 2500) {
        position = LiftPosition.BOTTOM
        isBusy = true
        val elapsedTime = ElapsedTime()
        while (!isLowered && !linearOpMode.isStopRequested && elapsedTime.milliseconds() < timeout) {
            setPower(-0.30)
        }
        resetEncoder()
        setPower(0.0)
        isBusy = false
    }

    fun startSettingMotorPowers() {
        thread(start = true) {
            while (linearOpMode.opModeIsActive()) {
                if (shouldHoldLiftPosition && !isBusy) {
                    setPower(powerController.outputPower)
                }
            }
        }
    }

    fun resetEncoder() {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    companion object {
        private const val MINIMUM_POWER = -1.00
        private const val MAXIMUM_POWER = 1.00

        private val PID_COEFFICIENTS =
            PIDCoefficients().also {
                it.p = 0.005
                it.i = 0.0005
                it.d = 0.0
            }
    }

}
