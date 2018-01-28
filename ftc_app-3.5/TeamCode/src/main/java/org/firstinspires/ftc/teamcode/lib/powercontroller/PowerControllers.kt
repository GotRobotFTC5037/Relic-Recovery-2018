package org.firstinspires.ftc.teamcode.lib.powercontroller

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.concurrent.thread

typealias Power = Double

/**
 * A interface for power controllers.
 */
interface PowerController {
    var target: Double
    var inputValueHandler: () -> Double
    val output: Power
}

/**
 * A PowerController that always returns the same value.
 */
class StaticPowerController(private val power: Power): PowerController {

    // Changing these values does nothing. Output is always the same.
    override var target = 0.0
    override var inputValueHandler = { 0.0 }

    override val output: Power
        get() = power
}

/**
 * A PowerController that returns a value proportional to the difference between the input value
 * and target value.
 */
class ProportionalPowerController(
    private var gain: Double,
    override var target: Double = 0.0,
    override var inputValueHandler: () -> Double = { 0.0 }
): PowerController {
    override val output: Power
        get() {
            val input = inputValueHandler()
            val error = target - input
            return error * gain
        }
}

/**
 * A PowerController that returns a value controlled by PID.
 */
class PIDPowerController(
    private val linearOpMode: LinearOpMode,
    private val coefficients: PIDCoefficients,
    override var target: Double = 0.0,
    override var inputValueHandler: () -> Double = {0.0}
) : PowerController {

    private lateinit var updateThread: Thread

    private var previousError = 0.0
    private var runningIntegral = 0.0
    private val lastUpdateElapsedTime: ElapsedTime by lazy { ElapsedTime() }

    private var proportionalOutput = 0.0
    private var integralOutput = 0.0
    private var derivativeOutput = 0.0
    private var pidOutput = 0.0

    override val output: Power
        get() = pidOutput

    init {
        startUpdatingOutput()
    }

    private fun startUpdatingOutput() {
        updateThread = thread(start = true) {
            while (linearOpMode.opModeIsActive() && !Thread.interrupted()) {
                val dt = lastUpdateElapsedTime.milliseconds() / 1000

                val error = target - inputValueHandler()
                runningIntegral += error * dt
                val derivative = (previousError - error) / dt

                proportionalOutput = error * coefficients.p
                integralOutput = runningIntegral * coefficients.i
                derivativeOutput = derivative * coefficients.d
                pidOutput = proportionalOutput + integralOutput + derivativeOutput

                lastUpdateElapsedTime.reset()
            }
        }
    }

    fun stopUpdatingOutput() {
        if (::updateThread.isInitialized && updateThread.isAlive) {
            updateThread.interrupt()
        }
    }

}