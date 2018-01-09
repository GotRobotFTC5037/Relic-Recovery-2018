package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import java.lang.Math.pow
import java.lang.Math.signum
import kotlin.concurrent.thread
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class RelicRecoveryTeleOp : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "TeleOp"
    }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robotInUse = RelicRecoveryRobotOpModeManager.robotInUse as RelicRecoveryRobot?
        RelicRecoveryRobotOpModeManager.robotInUse = null

        val robot = if (robotInUse != null) {
            robotInUse.linearOpMode = this
            robotInUse
        } else {
            val newRobot = RelicRecoveryRobot()
            newRobot.linearOpMode = this
            newRobot.setup(hardwareMap)
            newRobot
        }

        robot.shouldCorrectHeading = false
        robot.waitForGyroCalibration()
        waitForStart()
        robot.start()
        robot.startUpdatingRangeSensors()

        robot.raiseJewelStick()

        while (opModeIsActive()) {

            // Gamepad 1: Movement
            when {
                gamepad1.dpad_up -> { robot.setDrivePower(0.20); if (!robot.shouldCorrectHeading) { robot.targetHeading = robot.heading; robot.shouldCorrectHeading = true }}
                gamepad1.dpad_down -> { robot.setDrivePower(-0.20); if (!robot.shouldCorrectHeading) { robot.targetHeading = robot.heading; robot.shouldCorrectHeading = true }}
                gamepad1.dpad_left -> { robot.setStrafePower(-0.25); if (!robot.shouldCorrectHeading) { robot.targetHeading = robot.heading; robot.shouldCorrectHeading = true }}
                gamepad1.dpad_right -> { robot.setStrafePower(0.25); if (!robot.shouldCorrectHeading) { robot.targetHeading = robot.heading; robot.shouldCorrectHeading = true }}

                else -> {
                    val x = gamepad1.left_stick_x.toDouble() * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 1.0 else 0.5
                    val y = gamepad1.left_stick_y.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 1.0 else 0.85
                    val z = gamepad1.right_stick_x.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 1.0 else 0.75

                    val xPower = abs(pow(x, 3.0)) * signum(x)
                    val yPower = abs(pow(y, 3.0)) * signum(y)
                    val zPower = (abs(pow(z, 3.0)) * signum(z))

                    if (zPower == 0.0) {
                        if (!robot.shouldCorrectHeading) {
                            robot.targetHeading = robot.heading
                            robot.shouldCorrectHeading = true
                        }
                    } else {
                        robot.shouldCorrectHeading = false
                    }

                    robot.setDirection(xPower, yPower, zPower)
                }
            }

            when {
                gamepad1.a -> robot.turn(0.75, 90.0)
                gamepad1.b -> robot.turn(0.75, 180.0)
                gamepad1.y -> robot.turn(0.75, 0.0)
                gamepad1.x -> robot.turn(0.75, -90.0)
            }

            // Gamepad 2: Attachments
            robot.lift.manuallyMove((gamepad2.left_stick_y * -1).toDouble())

            when {
                gamepad2.dpad_up -> thread(start = true) { robot.lift.moveUpLevel() }
                gamepad2.dpad_down -> thread(start = true) { robot.lift.moveDownLevel() }
            }

            when {
                gamepad2.a -> { robot.closeGlyphGrabbers(); robot.retractGlyphDeployer() }
                gamepad2.y -> { robot.releaseGlyphGrabbers(); robot.extendGlyphDeployer() }
                gamepad2.b -> { robot.openGlyphGrabbers(); robot.retractGlyphDeployer() }
                gamepad2.x -> { robot.smallOpenGlyphGrabbers(); robot.retractGlyphDeployer() }
            }

            // Telemetry
            telemetry.addLine("FL: ${robot.frontLeftRangeSensor.distanceDetected}; FR; ${robot.frontRightRangeSensor.distanceDetected}")
            telemetry.addLine("SL: ${robot.leftRangeSensor.distanceDetected}; SR: ${robot.rightRangeSensor.distanceDetected}")
            telemetry.addLine()
            telemetry.addLine("Lift: ${robot.lift.motor.currentPosition}")
            telemetry.update()
        }

        robot.stopAllDriveMotors()
    }

}
