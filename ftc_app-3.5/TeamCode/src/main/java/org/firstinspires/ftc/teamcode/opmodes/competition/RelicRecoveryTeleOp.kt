package org.firstinspires.ftc.teamcode.opmodes.competition

import RelicRecoveryRobotOpModeManager
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot
import java.lang.Math.pow
import java.lang.Math.signum
import kotlin.math.abs

@TeleOp(name = "TeleOp")
class RelicRecoveryTeleOp : LinearOpMode() {

    companion object {
        val OPMODE_NAME = "TeleOp"
    }

    private var liftPosition = 0

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robotInUse = RelicRecoveryRobotOpModeManager.robotInUse as RelicRecoveryRobot?
        RelicRecoveryRobotOpModeManager.robotInUse = null

        val robot = if (robotInUse != null) {
            robotInUse.liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            robotInUse
        } else {
            val newRobot = RelicRecoveryRobot()
            newRobot.linearOpMode = this
            newRobot.setup(hardwareMap)
            newRobot.liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            newRobot
        }

        robot.linearOpMode = this
        robot.shouldCorrectHeading = false
        robot.waitForGyroCalibration()
        waitForStart()
        robot.start()
        robot.startUpdatingRangeSensors()

        robot.raiseJewelStick()

        while (opModeIsActive()) {
            robot.liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

            // Gamepad 1: Movement
            when {
                gamepad1.dpad_up -> { robot.setDrivePower(0.20); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }
                gamepad1.dpad_down -> { robot.setDrivePower(-0.20); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }
                gamepad1.dpad_left -> { robot.setStrafePower(-0.25); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }
                gamepad1.dpad_right -> { robot.setStrafePower(0.25); robot.shouldCorrectHeading = true; robot.targetHeading = robot.heading }

                else -> {
                    robot.shouldCorrectHeading = false
                    val x = gamepad1.left_stick_x.toDouble() * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 1.0 else 0.5
                    val y = gamepad1.left_stick_y.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 1.0 else 0.85
                    val z = gamepad1.right_stick_x.toDouble() * -1.0 * if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) 1.0 else 0.75

                    val xPower = abs(pow(x, 3.0)) * signum(x)
                    val yPower = abs(pow(y, 3.0)) * signum(y)
                    val zPower = (abs(pow(z, 3.0)) * signum(z))

                    robot.setDirection(xPower, yPower, zPower)
                }
            }

            // Gamepad 2: Attachments
            val winchPower = (gamepad2.left_stick_y * -1).toDouble()
            robot.setLiftWinchPower(winchPower)

            /*when {
                gamepad2.dpad_up -> {
                    if (liftPosition != 4) {
                        liftPosition += 1
                    }
                    while (gamepad2.dpad_up) {
                        sleep(10)
                    }
                }

                gamepad2.dpad_down -> {
                    if (liftPosition != 0) {
                        liftPosition -= 1
                    }
                    while (gamepad2.dpad_down) {
                        sleep(10)
                    }
                }
            }

            when (liftPosition) {
                0 -> robot.setLiftHeight(RelicRecoveryRobot.LiftPosition.BOTTOM_LEVEL)
                1 -> robot.setLiftHeight(RelicRecoveryRobot.LiftPosition.FIRST_LEVEL)
                2 -> robot.setLiftHeight(RelicRecoveryRobot.LiftPosition.SECOND_LEVEL)
                3 -> robot.setLiftHeight(RelicRecoveryRobot.LiftPosition.THIRD_LEVEL)
                4 -> robot.setLiftHeight(RelicRecoveryRobot.LiftPosition.FOURTH_LEVEL)
            }*/

            when {
                gamepad2.a -> { robot.closeGlyphGrabbers(); robot.retractGlyphDeployer() }
                gamepad2.y -> { robot.releaseGlyphGrabbers(); robot.extendGlyphDeployer() }
                gamepad2.b -> { robot.openGlyphGrabbers(); robot.retractGlyphDeployer() }
                gamepad2.x -> { robot.smallOpenGlyphGrabbers(); robot.retractGlyphDeployer() }
            }

            // Information
            telemetry.addLine("FL: ${robot.frontLeftRangeSensor.distanceDetected}; FR; ${robot.frontRightRangeSensor.distanceDetected}")
            telemetry.addLine("SL: ${robot.leftRangeSensor.distanceDetected}; SR: ${robot.rightRangeSensor.distanceDetected}")
            telemetry.addLine()
            telemetry.addLine("Lift: ${robot.liftMotor.currentPosition}")
            telemetry.update()
        }

        robot.stopAllDriveMotors()
    }

}
