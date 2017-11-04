package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libraries.PictographIdentifier;

public class RelicRecoveryRobot extends MecanumRobot {
    public DcMotor mWinchMotor;
    private Servo mJewelStick;

    private ColorSensor mColorSensor;

    public PictographIdentifier pictographIdentifier = new PictographIdentifier();

    public void setup(LinearOpMode linearOpMode) {
        super.setup(linearOpMode);

        mWinchMotor = linearOpMode.hardwareMap.dcMotor.get("winch motor");
        mWinchMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        mJewelStick = linearOpMode.hardwareMap.servo.get("jewel stick");

        mColorSensor = linearOpMode.hardwareMap.colorSensor.get("color sensor");

        pictographIdentifier.activate();
    }

    public void setWinchPower(double power) {
        mWinchMotor.setPower(power);
    }

    public void setJewelStickPosition(double position) {
        mJewelStick.setPosition(position);
    }

    public void lowerJewelStick() {
        setJewelStickPosition(1.0);
    }

    public void raiseJewelStick() {
        setJewelStickPosition(0.0);
    }

    public ColorSensor getColorSensor() {
        return mColorSensor;
    }
}
