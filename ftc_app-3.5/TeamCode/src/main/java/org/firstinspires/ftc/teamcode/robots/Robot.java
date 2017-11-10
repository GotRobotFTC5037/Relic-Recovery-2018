package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Robot {
	public LinearOpMode linearOpMode;

	public enum AllianceColor {
		RED, BLUE, UNKNOWN
	}

    public abstract void setup(LinearOpMode linearOpMode);

    public void turn(double power, int degrees) {
	    if(degrees > 0) {
		    double targetHeading = getGyroHeading() + degrees;
		    setTurnPower(Math.abs(power));
		    while(getGyroHeading() < targetHeading && linearOpMode.opModeIsActive()) {
			    linearOpMode.telemetry.addData("Heading", getGyroHeading());
			    linearOpMode.telemetry.update();
			    linearOpMode.idle();
		    }
	    } else if(degrees < 0) {
		    double targetHeading = getGyroHeading() + degrees;
		    setTurnPower(-Math.abs(power));
		    while(getGyroHeading() > targetHeading && linearOpMode.opModeIsActive()) {
			    linearOpMode.telemetry.addData("-Heading", getGyroHeading());
			    linearOpMode.telemetry.update();
		    	linearOpMode.idle();
		    }
	    }

	    stopAllDriveMotors();
    }

    public abstract void setDrivePower(double power);
    public abstract void setTurnPower(double power);
    public abstract void stopAllDriveMotors();
    public abstract double getGyroHeading();
}
