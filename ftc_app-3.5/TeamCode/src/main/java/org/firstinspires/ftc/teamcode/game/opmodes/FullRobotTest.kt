package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.game.robots.Coda

@TeleOp(group = "Tests")
class FullRobotTest: LinearOpMode() {

    override fun runOpMode() {
        val robot = Coda(this)
        robot.setup()
        waitForStart()

        telemetry.log().add("Press A to perform drive motor tests.")
        waitForAButtonPress()

        robot.driveTrain.frontLeftMotor.power = 1.0
        waitForAButtonPress()

        robot.driveTrain.frontLeftMotor.power = 0.0
        robot.driveTrain.frontRightMotor.power = 1.0
        waitForAButtonPress()

        robot.driveTrain.frontRightMotor.power = 0.0
        robot.driveTrain.rearLeftMotor.power = 1.0
        waitForAButtonPress()

        robot.driveTrain.rearLeftMotor.power = 0.0
        robot.driveTrain.rearRightMotor.power = 1.0
        waitForAButtonPress()

        robot.driveTrain.rearRightMotor.power = 0.0
    }

    private fun waitForAButtonPress() {
        while(gamepad1.a && !isStopRequested) {
            idle()
        }
    }

}