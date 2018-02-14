package org.firstinspires.ftc.teamcode.lib.sensorfilter

interface SensorFilter {
    var sensorValueHandler: () -> Double
    val outputValue: Double
    fun stop()
}

enum class Direction {
    POSITIVE,
    NEGATIVE
}

class SingleDirectionSensorFilter(
    private val direction: Direction,
    override var sensorValueHandler: () -> Double
) : SensorFilter {

    private var outlayingValue = -1.0
    override val outputValue: Double
        get() {
            val sensorValue = sensorValueHandler()
            if (outlayingValue == -1.0) {
                outlayingValue = sensorValue
            } else {
                if (direction == Direction.POSITIVE && sensorValue > outlayingValue) {
                    outlayingValue = sensorValue
                } else if (direction == Direction.NEGATIVE && sensorValue < outlayingValue) {
                    outlayingValue = sensorValue
                }
            }

            return outlayingValue
        }

    override fun stop() {
        // Do nothing
    }
}