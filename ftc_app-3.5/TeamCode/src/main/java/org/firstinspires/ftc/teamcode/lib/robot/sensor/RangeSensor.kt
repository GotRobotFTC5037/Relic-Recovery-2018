package org.firstinspires.ftc.teamcode.lib.robot.sensor

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.I2cAddr
import kotlin.concurrent.thread

class RangeSensor(linearOpMode: LinearOpMode, name: String, address: I2cAddr = I2cAddr.create8bit(0x28), private val alpha: Double = 0.50): RobotSensor(linearOpMode) {

    companion object {
        private const val RAW_RANGE_VALUE_CUTOFF  = 200
    }

    private val sensor: ModernRoboticsI2cRangeSensor by lazy {
        val sensor = linearOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, name)
        sensor.i2cAddress = address
        sensor
    }

    var distanceDetected: Double = 0.0
    private set

    private lateinit var sensorUpdateThread: Thread

    fun startUpdatingDetectedDistance() {
        sensorUpdateThread = thread(start = true) {
            while (!linearOpMode.isStopRequested && !Thread.interrupted()) {
                val rawDistance = sensor.cmUltrasonic()
                if (rawDistance < RAW_RANGE_VALUE_CUTOFF) {
                    distanceDetected += (rawDistance - distanceDetected) * alpha
                }
                linearOpMode.sleep(50)
            }
        }
    }

    fun stopUpdatingDetectedDistance() {
        sensorUpdateThread.interrupt()
        sensorUpdateThread.join()
    }

}