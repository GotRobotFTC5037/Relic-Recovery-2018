package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Andrew on 10/21/17.
 */
@Autonomous
public class Autonomous_drive extends LinearOpMode {

    private DcMotor left_1;
    private DcMotor right_1;
    private DcMotor left_2;
    private DcMotor right_2;
    private ColorSensor sensorColor;
    private DistanceSensor sensorDistance;

    double power = .15;


    @Override
    public void runOpMode() throws InterruptedException {

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        right_1 = hardwareMap.get(DcMotor.class, "right_1");
        left_1 = hardwareMap.get(DcMotor.class, "left_1");
        right_2 = hardwareMap.get(DcMotor.class, "right_2");
        left_2 = hardwareMap.get(DcMotor.class, "left_2");
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        left_1.setDirection(DcMotor.Direction.REVERSE);
        right_1.setDirection(DcMotor.Direction.FORWARD);
        left_2.setDirection(DcMotor.Direction.REVERSE);
        right_2.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            if (sensorColor.blue()>sensorColor.red()) {
                right_1.setPower(-power);
                left_1.setPower(-power);
                right_2.setPower(-power);
                left_2.setPower(-power);
            }

            if (sensorColor.red()>sensorColor.blue()) {
                right_1.setPower(power);
                left_1.setPower(power);
                right_2.setPower(power);
                left_2.setPower(power);
            }
        }
    }
}