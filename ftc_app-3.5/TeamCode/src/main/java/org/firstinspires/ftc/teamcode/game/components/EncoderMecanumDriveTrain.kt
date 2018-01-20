package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrains.MecanumDriveTrain
import kotlin.math.abs

class EncoderMecanumDriveTrain(linearOpMode: LinearOpMode): MecanumDriveTrain(linearOpMode) {

    fun resetEncoders() {
        frontLeftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun getPosition(): RobotPosition {
        val frontLeftEncoderValue = frontLeftMotor.currentPosition
        val frontRightEncoderValue = frontRightMotor.currentPosition
        val backLeftEncoderValue = backLeftMotor.currentPosition
        val backRightEncoderValue = backRightMotor.currentPosition

        //val encoderPositionSet1 = frontLeftEncoderValue, backRightEncoderValue

        return RobotPosition(0.0, 0.0)
    }

    fun driveTo(targetPosition: RobotPosition, power: Double) {
        resetEncoders()
        if (targetPosition.y > 0)
            setDrivePower(power)
        else
            setDrivePower(-power)

        while (abs(getPosition().y) < abs(targetPosition.y)) {
            linearOpMode.sleep(10)
        }

        stop()
    }

}
