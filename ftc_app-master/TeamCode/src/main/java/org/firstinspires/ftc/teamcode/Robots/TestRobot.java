package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

public class TestRobot extends Robot {

    private DcMotor mLeftMotor;
    private DcMotor mRightMotor;
    private GyroSensor gyroSensor;

    private int lastRawGyroHeading_ = 0;
    private int gyroHeading_ = 0;


    public void setup(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;

        mLeftMotor = linearOpMode.hardwareMap.dcMotor.get("left motor");
        mLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mRightMotor = linearOpMode.hardwareMap.dcMotor.get("right motor");
        mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyroSensor = linearOpMode.hardwareMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();
    }

    public void setDrivePower(double power) {
        mLeftMotor.setPower(power);
        mRightMotor.setPower(power);
    }

    public void setTurnPower(double power) {
        mLeftMotor.setPower(power);
        mRightMotor.setPower(-power);
    }

    public void stop() {
        mLeftMotor.setPower(0);
        mRightMotor.setPower(0);
    }

    public double getGyroHeading() {
        int rawGyroHeading = gyroSensor.getHeading();

        if (rawGyroHeading - 180 > lastRawGyroHeading_) {
            gyroHeading_ = gyroHeading_ + rawGyroHeading - 360 - lastRawGyroHeading_;
        } else if (rawGyroHeading + 180 < lastRawGyroHeading_) {
            gyroHeading_ = gyroHeading_ + rawGyroHeading + 360 - lastRawGyroHeading_;
        } else {
            gyroHeading_ = gyroHeading_ + rawGyroHeading - lastRawGyroHeading_;
        }

        lastRawGyroHeading_ = rawGyroHeading;

        return Math.toRadians(gyroHeading_);
    }

}
