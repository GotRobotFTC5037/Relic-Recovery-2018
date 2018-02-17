package org.firstinspires.ftc.teamcode.lib.powercontroller

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.sign
import kotlin.properties.Delegates

typealias Power = Double

/**
 * A interface for creating classes that control the power of motors for specific types of movement.
 */
interface PowerController {
    var errorValueHandler: () -> Double
    val outputPower: Power
    fun stopUpdatingOutput()
}

/**
 * A PowerController that always returns the same value.
 */
class StaticPowerController(private val power: Power) : PowerController {
    override var errorValueHandler: () -> Double = { 0.0 }
    override val outputPower: Power
        get() = sign(errorValueHandler()) * abs(power)
    override fun stopUpdatingOutput() {
        // Does nothing
    }
}

/**
 * A PowerController that returns a value proportional to the difference between the input value
 * and target value.
 */
class ProportionalPowerController(private var gain: Double) : PowerController {
    override var errorValueHandler: () -> Double = { 0.0 }
    override val outputPower: Power
        get() = errorValueHandler() * gain
    override fun stopUpdatingOutput() {
        // Does nothing
    }
}

/**
 * A PowerController that returns a value controlled by PID.
 */
class PIDPowerController(
    private val linearOpMode: LinearOpMode,
    private val coefficients: PIDCoefficients,
    private val shouldPrintDebug: Boolean = false
) : PowerController {
    override var errorValueHandler: () -> Double by Delegates.observable({ 0.0 }) { _,_,_ ->
        resetIntegral()
    }
    private lateinit var updateThread: Thread
    private val lastUpdateElapsedTime: ElapsedTime by lazy { ElapsedTime() }
    private var previousError = -1.0
    private var runningIntegral = Double.POSITIVE_INFINITY
    private var pidOutput = 0.0

    override val outputPower: Power
        get() = pidOutput

    init {
        startUpdatingOutput()
    }

    private fun startUpdatingOutput() {
        previousError = -1.0
        resetIntegral()
        pidOutput = 0.0
        if (
            !::updateThread.isInitialized &&
            !linearOpMode.isStopRequested
        ) {
            updateThread = thread(start = true) {
                while (!linearOpMode.isStopRequested && !Thread.interrupted()) {
                    val error = errorValueHandler()
                    if (previousError != -1.0 && !runningIntegral.isInfinite()) {
                        val dt = lastUpdateElapsedTime.milliseconds() / 1000
                        runningIntegral += error * dt
                        val derivative = (previousError - error) / dt
                        val proportionalOutput = error * coefficients.p
                        val integralOutput = runningIntegral * coefficients.i
                        val derivativeOutput = derivative * coefficients.d
                        pidOutput = proportionalOutput + integralOutput + derivativeOutput

                        if (shouldPrintDebug) {
                            linearOpMode.telemetry.addLine("P: $proportionalOutput")
                            linearOpMode.telemetry.addLine("I: $integralOutput")
                            linearOpMode.telemetry.addLine("D: $derivativeOutput")
                            linearOpMode.telemetry.update()
                        }

                    }

                    previousError = error
                    lastUpdateElapsedTime.reset()
                    linearOpMode.sleep(1)
                }
            }
        }

        linearOpMode.telemetry.update()
    }

    private fun resetIntegral() {
        runningIntegral = 0.0
    }

    override fun stopUpdatingOutput() {
        if (::updateThread.isInitialized && updateThread.isAlive) {
            updateThread.interrupt()
            resetIntegral()
        }
    }

}
