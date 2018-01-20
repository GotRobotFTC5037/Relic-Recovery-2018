package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.libraries.robot.attachments.RobotAttachment

class RelicGrabber(linearOpMode: LinearOpMode) : RobotAttachment(linearOpMode) {

    val winch: DcMotor = linearOpMode.hardwareMap.dcMotor.get("left_motor")
    val cam: DcMotor = linearOpMode.hardwareMap.dcMotor.get("right_motor")
    val leftGrabber: Servo = linearOpMode.hardwareMap.servo.get("left_servo")
    val rightGrabber: Servo = linearOpMode.hardwareMap.servo.get("right_servo")

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