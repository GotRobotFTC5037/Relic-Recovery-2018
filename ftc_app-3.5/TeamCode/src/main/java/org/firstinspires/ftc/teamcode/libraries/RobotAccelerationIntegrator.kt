package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity

class RobotAccelerationIntegrator
internal constructor() : BNO055IMU.AccelerationIntegrator {

    private var parameters: BNO055IMU.Parameters? = null
    private var position: Position
    private var velocity: Velocity
    private var acceleration: Acceleration? = null

    override fun getPosition(): Position = this.position
    override fun getVelocity(): Velocity = this.velocity
    override fun getAcceleration(): Acceleration? = this.acceleration

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
                acceleration = linearAcceleration

                if (previousAcceleration!!.acquisitionTime != 0L) {
                    val deltaVelocity = meanIntegrate(acceleration!!, previousAcceleration)
                    velocity = plus(velocity, deltaVelocity)
                }

                if (previousVelocity.acquisitionTime != 0L) {
                    val deltaPosition = meanIntegrate(velocity, previousVelocity)
                    position = plus(position, deltaPosition)
                }

            } else {
                acceleration = linearAcceleration
            }

        }
    }
}
