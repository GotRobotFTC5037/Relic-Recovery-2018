package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity

class RobotAccelerationIntegrator
internal constructor() : BNO055IMU.AccelerationIntegrator {

    private var parameters: BNO055IMU.Parameters? = null
    private var position: Position
    private var velocity: Velocity
    private var acceleration: Acceleration? = null

    var nZeroAccelerationX = 0
    var nZeroAccelerationY = 0
    var nZeroAccelerationZ = 0

    init {
        this.parameters = null
        this.position = Position()
        this.velocity = Velocity()
        this.acceleration = null
    }

    override fun initialize(parameters: BNO055IMU.Parameters, initialPosition: Position?, initialVelocity: Velocity?) {
        this.parameters = parameters
        this.position = initialPosition ?: this.position
        this.velocity = initialVelocity ?: this.velocity
        this.acceleration = null
    }

    override fun update(linearAcceleration: Acceleration) {

        // We should always be given a timestamp here
        if (linearAcceleration.acquisitionTime != 0L) {

            // We can only integrate if we have a previous acceleration to baseline from
            if (acceleration != null) {
                val previousAcceleration = acceleration
                val previousVelocity = velocity

                // Remove the noise from the acceleration
                if(Math.abs(linearAcceleration.xAccel) <= NOISE_THRESHOLD) linearAcceleration.xAccel = 0.0
                if(Math.abs(linearAcceleration.yAccel) <= NOISE_THRESHOLD) linearAcceleration.yAccel = 0.0
                if(Math.abs(linearAcceleration.zAccel) <= NOISE_THRESHOLD) linearAcceleration.zAccel = 0.0
                acceleration = linearAcceleration

                // Count the number of times that the acceleration is a certain direction is zero.
                if(acceleration!!.xAccel == 0.0) nZeroAccelerationX++ else nZeroAccelerationX = 0
                if(acceleration!!.yAccel == 0.0) nZeroAccelerationY++ else nZeroAccelerationY = 0
                if(acceleration!!.zAccel == 0.0) nZeroAccelerationZ++ else nZeroAccelerationZ = 0

                // If the acceleration in a certain direction is 0 for long enough, we can
                // assume the robot stopped and set the velocity to 0.
                if(nZeroAccelerationX >= ZERO_ACCELERATION_COUNT_THRESHOLD) velocity.xVeloc = 0.0
                if(nZeroAccelerationY >= ZERO_ACCELERATION_COUNT_THRESHOLD) velocity.yVeloc = 0.0
                if(nZeroAccelerationZ >= ZERO_ACCELERATION_COUNT_THRESHOLD) velocity.zVeloc = 0.0

                // Calculate velocity
                if (previousAcceleration!!.acquisitionTime != 0L) {
                    val deltaVelocity = NavUtil.meanIntegrate(acceleration!!, previousAcceleration)
                    velocity = NavUtil.plus(velocity, deltaVelocity)
                }

                // Calculate position
                if (previousVelocity.acquisitionTime != 0L) {
                    val deltaPosition = NavUtil.meanIntegrate(velocity, previousVelocity)
                    position = NavUtil.plus(position, deltaPosition)
                }

            } else {
                acceleration = linearAcceleration
            }

        }
    }

    override fun getPosition(): Position = this.position
    override fun getVelocity(): Velocity = this.velocity
    override fun getAcceleration(): Acceleration? = this.acceleration

    companion object {
        val NOISE_THRESHOLD = 0.2
        val ZERO_ACCELERATION_COUNT_THRESHOLD = 25
    }
}
