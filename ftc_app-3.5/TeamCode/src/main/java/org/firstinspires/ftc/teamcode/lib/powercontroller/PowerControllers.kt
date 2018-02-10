package org.firstinspires.ftc.teamcode.lib.powercontroller

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.sign

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
    private val coefficients: PIDCoefficients
) : PowerController {
    override var errorValueHandler: () -> Double = { 0.0 }
    private lateinit var updateThread: Thread
    private val lastUpdateElapsedTime: ElapsedTime by lazy { ElapsedTime() }
    private var previousError = 0.0
    private var runningIntegral = 0.0
    private var pidOutput = 0.0

    override val outputPower: Power
        get() = pidOutput

    init {
        startUpdatingOutput()
    }

    private fun startUpdatingOutput() {
        if (
            !::updateThread.isInitialized &&
            !linearOpMode.isStopRequested
        ) {
            runningIntegral = 0.0
            previousError = 0.0
            updateThread = thread(start = true) {
                while (!linearOpMode.isStopRequested && !Thread.interrupted()) {
                    val dt = lastUpdateElapsedTime.milliseconds() / 1000
                    val error = errorValueHandler()
                    runningIntegral += error * dt
                    val derivative = (error - previousError) / dt
                    val proportionalOutput = error * coefficients.p
                    val integralOutput = runningIntegral * coefficients.i
                    val derivativeOutput = derivative * coefficients.d
                    pidOutput = proportionalOutput + integralOutput + derivativeOutput
                    previousError = error
                    lastUpdateElapsedTime.reset()

                    linearOpMode.sleep(1)
                }
            }
        }
    }

    override fun stopUpdatingOutput() {
        if (::updateThread.isInitialized && updateThread.isAlive) {
            updateThread.interrupt()
        }
    }

}
