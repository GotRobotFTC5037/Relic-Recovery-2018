package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap

abstract class Robot {

    enum class SetupPosition {
        FRONT_RED, FRONT_BLUE, BACK_BLUE, BACK_RED, UNKNOWN
    }

    lateinit var linearOpMode: LinearOpMode

    abstract fun setup(hardwareMap: HardwareMap)
    abstract fun setDrivePower(power: Double)
    abstract fun setTurnPower(power: Double)
    abstract fun stopAllDriveMotors()
    abstract fun turn(power: Double, degrees: Double)
}
