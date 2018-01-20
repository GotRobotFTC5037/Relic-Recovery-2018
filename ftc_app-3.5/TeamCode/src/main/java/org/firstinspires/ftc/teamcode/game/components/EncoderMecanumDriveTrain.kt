package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrains.MecanumDriveTrain

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
        return RobotPosition(0.0, 0.0)
    }

    fun driveTo(position: RobotPosition, power: Double) {

    }

}
