package org.firstinspires.ftc.teamcode.libraries.sensors

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.I2cAddr
import kotlin.concurrent.thread

class RangeSensor(private val linearOpMode: LinearOpMode, name: String, address: I2cAddr = I2cAddr.create8bit(0x28), private val alpha: Double = 0.75) {

    companion object {
        val RAW_RANGE_VALUE_CUTOFF  = 200
    }

    private val sensor = linearOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, name)

    init {
        sensor.i2cAddress = address
    }

    var distanceDetected: Double = 0.0
    private set

    fun startUpdatingDetectedDistance() {
        thread(start = true) {
            while (!linearOpMode.isStopRequested) {
                val rawDistance = sensor.cmUltrasonic()
                if (rawDistance < RAW_RANGE_VALUE_CUTOFF) {
                    distanceDetected += (rawDistance - distanceDetected) * alpha
                }
                linearOpMode.sleep(50)
            }
        }
    }

}