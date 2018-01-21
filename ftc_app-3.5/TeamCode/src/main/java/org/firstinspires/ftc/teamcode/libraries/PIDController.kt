package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.concurrent.thread

typealias PIDControllerUpdateHandler = () -> Double

class PIDController(
    val linearOpMode: LinearOpMode,
    val coefficients: PIDCoefficients
) {

    private lateinit var updateHandler: PIDControllerUpdateHandler
    private val updateHandlerIsSet: Boolean get() = ::updateHandler.isInitialized
    private val updateElapsedTime: ElapsedTime by lazy { ElapsedTime() }

    private var previousError = 0.0
    private var runningInterval: Double = 0.0

    var proportionalOutput = 0.0
    var integralOutput = 0.0
    var derivativeOutput = 0.0
    var output = 0.0

    fun start() {
        if (!updateHandlerIsSet)
            throw IllegalStateException("setUpdateHandler() must be called before calling start()")

        thread(start = true) {
            while (linearOpMode.opModeIsActive()) {
                val dt = updateElapsedTime.milliseconds() / 1000

                val error = updateHandler()
                runningInterval += error * dt
                val derivative = (previousError - error) / dt

                proportionalOutput = error * coefficients.p
                integralOutput = runningInterval * coefficients.i
                derivativeOutput = derivative * coefficients.d
                output = proportionalOutput + integralOutput + derivativeOutput

                updateElapsedTime.reset()
            }
        }
    }

    fun setUpdateHandler(handler: PIDControllerUpdateHandler) {
        updateHandler = handler
    }

}