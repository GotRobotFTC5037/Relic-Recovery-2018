package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

class TestRobot extends Robot {

    private DcMotor mLeftMotor;
    private DcMotor mRightMotor;
    private GyroSensor gyroSensor;

    private int lastRawGyroHeading_ = 0;
    private int gyroHeading_ = 0;

    @Override public void setup(HardwareMap hardwareMap) {
        mLeftMotor = hardwareMap.dcMotor.get("left motor");
        mLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mRightMotor = hardwareMap.dcMotor.get("right motor");
        mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        gyroSensor.calibrate();
    }

    void setDrivePower(double power) {
        mLeftMotor.setPower(power);
        mRightMotor.setPower(power);
    }

    void setTurnPower(double power) {
        mLeftMotor.setPower(power);
        mRightMotor.setPower(-power);
    }

    void turn(double power, double radians) {
        if(radians > 0) {
            double targetHeading = getGyroHeading() + radians;
            setTurnPower(Math.abs(power));
            while(getGyroHeading() < targetHeading) {
                Thread.yield();
            }
        } else if(radians < 0) {
            double targetHeading = getGyroHeading() - radians;
            setTurnPower(-Math.abs(power));
            while(getGyroHeading() > targetHeading) {
                Thread.yield();
            }
        }
    }

    double getGyroHeading() {
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
