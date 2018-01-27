package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.game.robots.Coda

@TeleOp(group = "Tests")
class MotorSpeedTest: LinearOpMode() {
    override fun runOpMode() {
        val robot = Coda(this)
        robot.setup()
        waitForStart()
        robot.driveTrain.linearTimeDrive(5000, 1.0)

        telemetry.addLine("Front Left: ${robot.driveTrain.frontLeftMotor.currentPosition}")
        telemetry.addLine("Rear Left: ${robot.driveTrain.rearLeftMotor.currentPosition}")
        telemetry.addLine("Front Right: ${robot.driveTrain.frontRightMotor.currentPosition}")
        telemetry.addLine("Rear Right: ${robot.driveTrain.rearRightMotor.currentPosition}")
        telemetry.update()

        robot.driveTrain.stop()

        while (opModeIsActive()) {
            sleep(10)
        }
    }
}