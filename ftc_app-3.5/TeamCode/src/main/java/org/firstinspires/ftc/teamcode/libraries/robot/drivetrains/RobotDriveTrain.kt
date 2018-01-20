package org.firstinspires.ftc.teamcode.libraries.robot.drivetrains

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.libraries.robot.RobotComponent

abstract class RobotDriveTrain(override val linearOpMode: LinearOpMode) : RobotComponent {

    abstract fun setDrivePower(power: Double)

    abstract fun setTurnPower(power: Double)

    abstract fun stop()

    open fun timeDrive(duration: Long, power: Double = 0.75) {
        setDrivePower(power)
        linearOpMode.sleep(duration)
        setDrivePower(0.0)
    }

}