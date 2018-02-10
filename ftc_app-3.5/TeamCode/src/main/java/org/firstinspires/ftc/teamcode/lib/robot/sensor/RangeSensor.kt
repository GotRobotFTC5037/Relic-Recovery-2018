package org.firstinspires.ftc.teamcode.lib.robot.sensor

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.I2cAddr
import kotlin.concurrent.thread

class RangeSensor(
    linearOpMode: LinearOpMode,
    name: String,
    address: I2cAddr = I2cAddr.create8bit(0x28),
    private val alpha: Double = 1.0
) : RobotSensor(linearOpMode) {

    companion object {
        private const val RAW_RANGE_VALUE_CUTOFF  = 200
    }

    private val sensor: ModernRoboticsI2cRangeSensor by lazy {
        val sensor = linearOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, name)
        sensor.i2cAddress = address
        sensor
    }

    var distanceDetected: Double = -1.0
    private set

    private lateinit var sensorUpdateThread: Thread

    fun startUpdatingDetectedDistance() {
        if (!::sensorUpdateThread.isInitialized || !sensorUpdateThread.isAlive) {
            sensorUpdateThread = thread(start = true) {
                while (linearOpMode.opModeIsActive() && !Thread.interrupted()) {
                    val rawDistance = sensor.rawUltrasonic()
                    if (distanceDetected == -1.0) {
                        distanceDetected = rawDistance.toDouble()
                        continue
                    }

                    if (rawDistance < RAW_RANGE_VALUE_CUTOFF) {
                        distanceDetected += (rawDistance - distanceDetected) * alpha
                    }
                }
            }
        }
    }

    fun stopUpdatingDetectedDistance() {
        sensorUpdateThread.interrupt()
        sensorUpdateThread.join()
    }

}