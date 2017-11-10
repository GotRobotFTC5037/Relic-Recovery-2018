package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@TeleOp(name = "TeleOp")
class RelicRecoveryTeleOp : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot
        robot.setup(this)
        robot.waitForGyroCalibration()

        waitForStart()

        robot.raiseJewelStick()

        while (opModeIsActive()) {
            val xPower = gamepad1.right_stick_x.toDouble()
            val yPower = (gamepad1.right_stick_y * -1).toDouble()
            val zPower: Double = if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                -Math.sqrt(2.0) / 2
            } else if (!gamepad1.left_bumper && gamepad1.right_bumper) {
                Math.sqrt(2.0) / 2
            } else {
                0.0
            }

            if (gamepad2.a) {
                robot.closeGlyphGrabbers()
            } else if (gamepad2.b) {
                robot.openGlyphGrabbers()
            }

            robot.setDirection(xPower, yPower, zPower)

            val winchPower = (gamepad2.left_stick_y * -1).toDouble()
            if(true){//(winchPower < 0 && !robot.liftLimitSwitch.state) || winchPower > 0) {
                robot.setWinchPower(winchPower)
            } else {
                robot.setWinchPower(0.0)
            }

            idle()
        }

        robot.stopAllDriveMotors()
    }

}
