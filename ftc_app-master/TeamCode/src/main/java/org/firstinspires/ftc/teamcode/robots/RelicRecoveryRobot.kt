package org.firstinspires.ftc.teamcode.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.librarys.PictographIdentifier

class RelicRecoveryRobot: MecanumRobot() {
    lateinit var mWinchMotor: DcMotor
    val pictographIdentifier = PictographIdentifier()

    override fun setup(linearOpMode: LinearOpMode) {
        super.setup(linearOpMode)

        mWinchMotor = linearOpMode.hardwareMap.dcMotor.get("winch motor")
        mWinchMotor.direction = DcMotorSimple.Direction.FORWARD

        pictographIdentifier.activate()
    }

    fun setWinchPower(power: Double) {
        mWinchMotor.power = power
    }
}