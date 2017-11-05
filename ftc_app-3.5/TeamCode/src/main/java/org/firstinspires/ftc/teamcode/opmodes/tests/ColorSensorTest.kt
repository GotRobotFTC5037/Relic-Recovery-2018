package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.ColorSensor

@Disabled
@Autonomous
class ColorSensorTest: LinearOpMode() {

    override fun runOpMode() {
        val colorSensor: ColorSensor = hardwareMap.colorSensor.get("color sensor")
        colorSensor.enableLed(true)
        waitForStart()

        while(opModeIsActive()) {
            telemetry.addData("red", colorSensor.red())
            telemetry.addData("blue", colorSensor.blue())
            telemetry.update()
        }
    }

}