package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.Velocity

class RobotAccelerationIntegrator : BNO055IMU.AccelerationIntegrator {

    private var acceleration: Acceleration? = null

    override fun update(linearAcceleration: Acceleration?) {
        acceleration = linearAcceleration
    }

    override fun getAcceleration(): Acceleration {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getVelocity(): Velocity {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun getPosition(): Position {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun initialize(parameters: BNO055IMU.Parameters, initialPosition: Position?, initialVelocity: Velocity?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }
}