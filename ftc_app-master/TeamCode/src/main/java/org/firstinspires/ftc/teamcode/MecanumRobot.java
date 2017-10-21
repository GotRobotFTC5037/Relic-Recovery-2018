package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.abs;

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

    public void setDirection(double x, double y) {
        double powerSum = abs(x) + abs(y);
        double xPower = x / powerSum;
        double yPower = y / powerSum;

        mFrontLeftMotor.setPower(yPower - xPower);
        mFrontRightMotor.setPower(yPower + xPower);
        mBackLeftMotor.setPower(yPower + xPower);
        mBackRightMotor.setPower(yPower - xPower);
    }

    public void setTurnPower(double power) {
        mFrontLeftMotor.setPower(-power);
        mFrontRightMotor.setPower(power);
        mBackLeftMotor.setPower(-power);
        mBackRightMotor.setPower(power);
    }

}
