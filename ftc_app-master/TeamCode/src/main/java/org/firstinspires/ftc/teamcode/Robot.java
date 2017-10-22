package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

abstract class Robot {
    abstract void setup(HardwareMap hardwareMap);
    abstract void setDrivePower(double power);
    abstract void setTurnPower(double power);
    abstract void turn(double power, double radians);
    abstract double getGyroHeading();
}
