package org.firstinspires.ftc.teamcode.libraries

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.I2cAddr

class PixyCam(hardwareMap: HardwareMap, name: String, port: I2cAddr) {

    val i2cDevice = hardwareMap.i2cDeviceSynch.get(name)

}