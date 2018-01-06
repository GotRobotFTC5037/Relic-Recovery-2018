package org.firstinspires.ftc.teamcode.libraries.sensors

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.concurrent.thread

class RangeSensor(private val linearOpMode: LinearOpMode, name: String, private val alpha: Double = 0.75) {

    private val sensor = linearOpMode.hardwareMap.get(ModernRoboticsI2cRangeSensor::class.java, name)

    var distanceDetected: Double = 0.0
    private set

    fun startUpdatingDetectedDistance() {
        thread(start = true) {
            while (!linearOpMode.isStopRequested) {
                val rawDistance = sensor.cmUltrasonic()
                distanceDetected += (rawDistance - distanceDetected) * alpha
                linearOpMode.sleep(50)
            }
        }
    }

}