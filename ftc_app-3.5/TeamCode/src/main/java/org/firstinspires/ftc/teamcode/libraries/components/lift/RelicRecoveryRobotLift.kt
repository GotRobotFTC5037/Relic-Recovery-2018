package org.firstinspires.ftc.teamcode.libraries.components.lift

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs

class RelicRecoveryRobotLift(linearOpMode: LinearOpMode, motorName: String, direction: DcMotorSimple.Direction, binaryLimitDeviceName: String) : LimitedRobotLift(linearOpMode, motorName, direction, binaryLimitDeviceName) {

    /**
     * A position to set the lift to.
     */
    enum class LiftPosition(val encoderPosition: Int) {
        FOURTH_LEVEL(2950),
        THIRD_LEVEL(2000),
        SECOND_LEVEL(1150),
        FIRST_LEVEL(100),
        BOTTOM_LEVEL(0)
    }

    init {
        positionCorrectionCoefficient = 0.005
    }

    private var hasManuallyMovedLiftSinceUpdate = false
    private var currentLiftPosition = LiftPosition.BOTTOM_LEVEL

    override fun manuallyMove(power: Double) {
        hasManuallyMovedLiftSinceUpdate = true
        super.manuallyMove(power)
        updateCurrentLiftPosition()
    }

    override fun setPosition(targetPosition: Int, power: Double) {
        hasManuallyMovedLiftSinceUpdate = true
        super.setPosition(targetPosition, power)
    }

    fun setPosition(targetPosition: LiftPosition, power: Double = 0.5) {
        setPosition(targetPosition.encoderPosition, power)
        hasManuallyMovedLiftSinceUpdate = false
        currentLiftPosition = targetPosition
    }

    override fun drop(power: Double) {
        setPosition(LiftPosition.BOTTOM_LEVEL)
    }

    private fun updateCurrentLiftPosition() {
        val currentPosition = motor.currentPosition

        var closestLiftPosition: LiftPosition = LiftPosition.BOTTOM_LEVEL

        LiftPosition.values()
                .asSequence()
                .filter { abs(closestLiftPosition.encoderPosition - currentPosition) > abs(it.encoderPosition - currentPosition) }
                .forEach { closestLiftPosition = it }

        currentLiftPosition = closestLiftPosition
        hasManuallyMovedLiftSinceUpdate = false
    }

    fun moveUpLevel() {
        updateCurrentLiftPosition()

        if (currentLiftPosition.ordinal + 1 < LiftPosition.values().count()) {
            currentLiftPosition = LiftPosition.values()[currentLiftPosition.ordinal + 1]
        }

        setPosition(currentLiftPosition.encoderPosition)
    }

    fun moveDownLevel() {
        updateCurrentLiftPosition()

        if (currentLiftPosition.ordinal > 0) {
            currentLiftPosition = LiftPosition.values()[currentLiftPosition.ordinal - 1]
        }

        setPosition(currentLiftPosition.encoderPosition)
    }
}