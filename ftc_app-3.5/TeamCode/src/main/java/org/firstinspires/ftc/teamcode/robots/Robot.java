package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Robot {
	LinearOpMode linearOpMode;

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

	    stop();
    }

    public abstract void setDrivePower(double power);
    public abstract void setTurnPower(double power);
    public abstract void stop();
    public abstract double getGyroHeading();
}
