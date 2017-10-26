package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MecanumRobot extends Robot {

    private DcMotor mFrontLeftMotor;
    private DcMotor mFrontRightMotor;
    private DcMotor mBackLeftMotor;
    private DcMotor mBackRightMotor;

    private LinearOpMode linearOpMode;

    public void setup(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;

        mFrontLeftMotor = linearOpMode.hardwareMap.dcMotor.get("front left motor");
        mFrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mFrontRightMotor = linearOpMode.hardwareMap.dcMotor.get("front right motor");
        mFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        mBackLeftMotor = linearOpMode.hardwareMap.dcMotor.get("back left motor");
        mBackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mBackRightMotor = linearOpMode.hardwareMap.dcMotor.get("back right motor");
        mBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void turn(double power, double radians) {
        // TODO: Add code
    }

    public void setDrivePower(double power) {
        mFrontLeftMotor.setPower(power);
        mBackLeftMotor.setPower(power);
        mFrontRightMotor.setPower(power);
        mBackRightMotor.setPower(power);
    }

    public void setTurnPower(double power) {
        mFrontLeftMotor.setPower(power);
        mBackLeftMotor.setPower(power);
        mFrontRightMotor.setPower(-power);
        mBackRightMotor.setPower(-power);
    }

    public void stop() {
        mFrontLeftMotor.setPower(0);
        mBackLeftMotor.setPower(0);
        mFrontRightMotor.setPower(0);
        mBackRightMotor.setPower(0);
    }

    public double getGyroHeading() {
        // TODO: Add code
        return 0;
    }

    public void setDirection(double x, double y) {
        double wheelSet1ComponentPower = (y + x) / 2;
        double wheelSet2ComponentPower = (y - x) / 2;

        double wheelSet1Power = Math.sqrt(2 * Math.pow(wheelSet1ComponentPower, 2)) *
                (wheelSet1ComponentPower / Math.abs(wheelSet1ComponentPower));
        double wheelSet2Power = Math.sqrt(2 * Math.pow(wheelSet2ComponentPower, 2)) *
                (wheelSet2ComponentPower / Math.abs(wheelSet2ComponentPower));

        mFrontRightMotor.setPower(wheelSet1Power);
        mBackLeftMotor.setPower(wheelSet1Power);
        mFrontLeftMotor.setPower(wheelSet2Power);
        mBackRightMotor.setPower(wheelSet2Power);
    }

}
