package org.firstinspires.ftc.teamcode.lib.powerultil

import com.qualcomm.robotcore.util.Range
import kotlin.math.abs

class PowerAmplitudeMinimumClip(private val minimumPowerAmplitude: Double) {

    fun getOutputPower(input: Double): Double {
        return when {
            input > 0 -> Range.clip(input, abs(minimumPowerAmplitude), 1.0)
            input < 0 -> Range.clip(input, -1.0, -abs(minimumPowerAmplitude))
            else -> 0.0
        }
    }

}