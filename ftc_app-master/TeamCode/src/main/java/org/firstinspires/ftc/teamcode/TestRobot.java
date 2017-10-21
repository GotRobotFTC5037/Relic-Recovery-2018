package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestRobot extends Robot {

    private DcMotor mLeftMotor;
    private DcMotor mRightMotor;



    @Override public void setup(HardwareMap hardwareMap) {
        mLeftMotor = hardwareMap.dcMotor.get("left motor");
        mLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mRightMotor = hardwareMap.dcMotor.get("right motor");
        mRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDrivePower(double power) {
        mLeftMotor.setPower(power);
        mRightMotor.setPower(power);
    }

    public void setTurnPower(double power) {
        // TODO: Add code here
    }

}
