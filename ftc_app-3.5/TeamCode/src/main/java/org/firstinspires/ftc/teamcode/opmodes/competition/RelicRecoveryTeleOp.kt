package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@TeleOp(name = "TeleOp")
class RelicRecoveryTeleOp : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot
        robot.linearOpMode = this
        robot.setup(hardwareMap)

        robot.colorBeacon.yellow()
        robot.waitForGyroCalibration()
        robot.colorBeacon.green()

        waitForStart()
        robot.colorBeacon.blue()

        robot.raiseJewelStick()

        while (opModeIsActive()) {
            val xPower = gamepad1.left_stick_x.toDouble()
            val yPower = (gamepad1.left_stick_y * -1).toDouble()
            val zPower = (gamepad1.right_stick_x * -1).toDouble()
            val winchPower = (gamepad2.left_stick_y * -1).toDouble()

            if (gamepad2.a) { robot.closeGlyphGrabbers() }
            else if (gamepad2.b) { robot.openGlyphGrabbers() }
            robot.setDirection(xPower, yPower, zPower)
            robot.setLiftWinchPower(winchPower)
        }

        robot.stopAllDriveMotors()
    }

}
