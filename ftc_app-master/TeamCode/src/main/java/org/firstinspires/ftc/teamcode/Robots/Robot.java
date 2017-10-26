package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Robot {
    public abstract void setup(LinearOpMode linearOpMode);
    public abstract void turn(double power, double radians);
    public abstract void setDrivePower(double power);
    public abstract void setTurnPower(double power);
    public abstract void stop();
    public abstract double getGyroHeading();
}
