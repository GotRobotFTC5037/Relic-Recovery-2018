package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.libraries.robot.attachments.RobotAttachment

class RelicGrabber(linearOpMode: LinearOpMode) : RobotAttachment(linearOpMode) {

    val winch: DcMotor = linearOpMode.hardwareMap.dcMotor.get("X Motor")
    val cam: DcMotor = linearOpMode.hardwareMap.dcMotor.get("Y Motor")
    val leftGrabber: Servo = linearOpMode.hardwareMap.servo.get("X Servo")
    val rightGrabber: Servo = linearOpMode.hardwareMap.servo.get("Y Servo")

    init {
        winch.direction = DcMotorSimple.Direction.FORWARD
        cam.direction = DcMotorSimple.Direction.FORWARD
        leftGrabber.direction = Servo.Direction.FORWARD
        rightGrabber.direction = Servo.Direction.FORWARD
    }

    fun setGrabberPosistion(position: Double) {
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