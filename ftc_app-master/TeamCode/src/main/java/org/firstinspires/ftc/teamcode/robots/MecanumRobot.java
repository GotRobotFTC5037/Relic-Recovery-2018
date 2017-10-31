package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

public class MecanumRobot extends Robot {

	private DcMotor mFrontLeftMotor;
	private DcMotor mFrontRightMotor;
	private DcMotor mBackLeftMotor;
	private DcMotor mBackRightMotor;

	private GyroSensor mGyroSensor;

	private int lastRawGyroHeading_ = 0;
	private int gyroHeading_ = 0;


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



		mGyroSensor = linearOpMode.hardwareMap.gyroSensor.get("gyro");
		mGyroSensor.calibrate();


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
		int rawGyroHeading = mGyroSensor.getHeading();

		if (rawGyroHeading - 180 > lastRawGyroHeading_) {
			gyroHeading_ = gyroHeading_ + rawGyroHeading - 360 - lastRawGyroHeading_;
		} else if (rawGyroHeading + 180 < lastRawGyroHeading_) {
			gyroHeading_ = gyroHeading_ + rawGyroHeading + 360 - lastRawGyroHeading_;
		} else {
			gyroHeading_ = gyroHeading_ + rawGyroHeading - lastRawGyroHeading_;
		}

		lastRawGyroHeading_ = rawGyroHeading;

		return gyroHeading_;
	}

	public void setDirection(double x, double y, double z) {
		double wheelSet1ComponentPower = (y - x) / 2;
		double wheelSet2ComponentPower = (y + x) / 2;

		double wheelSet1Power = Math.sqrt(2 * Math.pow(wheelSet1ComponentPower, 2)) *
				Math.signum(wheelSet1ComponentPower);
		double wheelSet2Power = Math.sqrt(2 * Math.pow(wheelSet2ComponentPower, 2)) *
				Math.signum(wheelSet2ComponentPower);

		mFrontRightMotor.setPower(wheelSet1Power - z);
		mBackLeftMotor.setPower(wheelSet1Power + z);
		mFrontLeftMotor.setPower(wheelSet2Power + z);
		mBackRightMotor.setPower(wheelSet2Power - z);
	}
}

