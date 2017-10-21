package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

@TeleOp
public class PixyCamTest extends LinearOpMode {

    private I2cDeviceSynchSimple pixyCam;

    @Override public void runOpMode() throws InterruptedException {
        pixyCam = hardwareMap.i2cDeviceSynch.get("pixy cam");
        I2cAddr addr = new I2cAddr(0x54);
        pixyCam.setI2cAddress(addr);
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("2", pixyCam.read(0x51, 1)[0]);
            telemetry.update();
            idle();
        }

    }
}
