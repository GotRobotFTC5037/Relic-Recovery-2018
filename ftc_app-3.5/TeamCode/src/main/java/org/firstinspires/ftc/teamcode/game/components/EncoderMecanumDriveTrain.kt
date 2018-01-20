package org.firstinspires.ftc.teamcode.game.components

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.libraries.robot.drivetrains.MecanumDriveTrain
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt

class EncoderMecanumDriveTrain(linearOpMode: LinearOpMode): MecanumDriveTrain(linearOpMode) {

    var wheelDiameter = 10.0

    private val wheelCircumference: Double
        get() = wheelDiameter * PI

    private fun resetEncoders() {
        frontLeftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backLeftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        backRightMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        frontLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        frontRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        backRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun getPosition(): RobotPosition {
        val frontLeftEncoderValue = frontLeftMotor.currentPosition.toDouble()
        val frontRightEncoderValue = frontRightMotor.currentPosition.toDouble()
        val backLeftEncoderValue = backLeftMotor.currentPosition.toDouble()
        val backRightEncoderValue = backRightMotor.currentPosition.toDouble()

        val encoderValue1: Double = (frontLeftEncoderValue + backRightEncoderValue) / 2
        val encoderValue2: Double = (frontRightEncoderValue + backLeftEncoderValue) / 2

        val m = sqrt(2.0) / 2
        val encoderValue1VectorComponent = encoderValue1 * m
        val encoderValue2VectorComponent = encoderValue2 * m

        val t = 1440 * wheelCircumference
        val y = (encoderValue1VectorComponent + encoderValue2VectorComponent) / t
        val x = (encoderValue2VectorComponent - encoderValue1VectorComponent) / t

        return RobotPosition(x, y)
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
