package org.firstinspires.ftc.teamcode.libraries.robot

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap

interface Component {

    val linearOpMode: LinearOpMode

    val hardwareMap: HardwareMap
        get() = linearOpMode.hardwareMap

}
