package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.lib.robot.attachment.RobotAttachment

class RelicGrabber(linearOpMode: LinearOpMode) : RobotAttachment(linearOpMode) {

    private val winch: DcMotor by lazy {
        hardwareMap.dcMotor.get("left_motor")
    }

    private val cam: DcMotor by lazy {
        hardwareMap.dcMotor.get("right_motor")
    }

    private val leftGrabber: Servo by lazy {
        hardwareMap.servo.get("left_servo")
    }

    private val rightGrabber: Servo by lazy {
        linearOpMode.hardwareMap.servo.get("right_servo")
    }

    fun setGrabberPosition(position: Double) {
        leftGrabber.position = position
        rightGrabber.position = position
    }

    fun setWinchPower(power: Double) {
        winch.power = power
    }

    fun setCamPower(power: Double) {
        cam.power = power
    }

}