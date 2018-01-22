package org.firstinspires.ftc.teamcode.lib.robot.lift

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel

open class LimitedRobotLift(linearOpMode: LinearOpMode,
                            motorName: String,
                            direction: DcMotorSimple.Direction,
                            limitDeviceName: String):
        RobotLift(linearOpMode, motorName, direction) {

    private val limitDevice: DigitalChannel = linearOpMode.hardwareMap.digitalChannel.get(limitDeviceName)

    init {
        limitDevice.mode = DigitalChannel.Mode.INPUT
    }

    override fun liftIsLowered() = !limitDevice.state
}