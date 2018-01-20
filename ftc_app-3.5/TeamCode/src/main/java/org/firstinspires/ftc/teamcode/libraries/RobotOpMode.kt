package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode

abstract class RobotOpMode : LinearOpMode() {

    enum class OpModeType {
        AUTONOMOUS,
        TELEOP
    }

    abstract val type: OpModeType

}