package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumRobot extends Robot {

    private static Double MAX_DIRECTION_VALUE = Math.sqrt(2);

    private DcMotor mFrontLeftMotor;
    private DcMotor mFrontRightMotor;
    private DcMotor mBackLeftMotor;
    private DcMotor mBackRightMotor;



    @Override public void setup(HardwareMap hardwareMap) {
        mFrontLeftMotor = hardwareMap.dcMotor.get("front left motor");
        mFrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mFrontRightMotor = hardwareMap.dcMotor.get("front right motor");
        mFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        mBackLeftMotor = hardwareMap.dcMotor.get("back left motor");
        mBackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mBackRightMotor = hardwareMap.dcMotor.get("back right motor");
        mBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }



    @Override void setDrivePower(double power) {
        // TODO: Add code
    }



    @Override void setTurnPower(double power) {
        // TODO: Add code
    }



    @Override void turn(double power, double radians) {
        // TODO: Add code
    }



    @Override double getGyroHeading() {
        // TODO: Add code
        return 0;
    }
}
