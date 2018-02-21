package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.lib.robot.sensor.RangeSensor

@Autonomous(group = "Tests")
@Disabled
class CodaDriveTrainStallTest : LinearOpMode() {

    override fun runOpMode() {

        val robot = Coda(this)
        robot.setup()

        waitForStart()

        robot.driveTrain.resetEncoders()

        val timer = ElapsedTime()

        var drivePower = 0.0

        while(opModeIsActive()) {
            drivePower = timer.milliseconds() / TEST_TIME_MILLISECONDS
            robot.driveTrain.linearDriveAtPower(drivePower)

            if(robot.driveTrain.currentLinearEncoderPosition() > TEST_COMPLETION_THRESHOLD) {
                break
            }

            telemetry.addLine("Drive Power: $drivePower")
            telemetry.update()
        }

        robot.driveTrain.stop()

        telemetry.addLine("Drive Power: $drivePower")
        telemetry.update()

        while(opModeIsActive()) {
            // Do nothing.
        }

    }

    companion object {
        private const val TEST_TIME_MILLISECONDS = 500000
        private const val TEST_COMPLETION_THRESHOLD = 100
    }

}

@Autonomous(group = "Tests")
class SensorTest : LinearOpMode() {

    override fun runOpMode() {
        val robot = Coda(this)
        robot.setup()

        val frontLeftRangeSensor = robot.components[Coda.FRONT_LEFT_RANGE_SENSOR] as RangeSensor
        val frontRightRangeSensor = robot.components[Coda.FRONT_RIGHT_RANGE_SENSOR] as RangeSensor
        val leftRangeSensor = robot.components[Coda.LEFT_RANGE_SENSOR] as RangeSensor
        val rightRangeSensor = robot.components[Coda.RIGHT_RANGE_SENSOR] as RangeSensor
        val backRangeSensor = robot.components[Coda.BACK_RANGE_SENSOR] as RangeSensor

        waitForStart()

        frontLeftRangeSensor.startUpdatingDetectedDistance()
        frontRightRangeSensor.startUpdatingDetectedDistance()
        leftRangeSensor.startUpdatingDetectedDistance()
        rightRangeSensor.startUpdatingDetectedDistance()
        backRangeSensor.startUpdatingDetectedDistance()

        while (opModeIsActive()) {
            telemetry.addLine("FLRS: ${frontLeftRangeSensor.distanceDetected}")
            telemetry.addLine("FRRS: ${frontRightRangeSensor.distanceDetected}")
            telemetry.addLine("RRS: ${rightRangeSensor.distanceDetected}")
            telemetry.addLine("LRS: ${leftRangeSensor.distanceDetected}")
            telemetry.addLine("BRS: ${backRangeSensor.distanceDetected}")
            telemetry.update()
        }
    }

}

@Autonomous(group = "Tests")
@Disabled
class GlyphGrabberServoTest : LinearOpMode() {
    override fun runOpMode() {
        val robot = Coda(this)
        robot.setup()

        waitForStart()

        waitForAButtonPress()

        robot.glyphGrabber.topLeftGlyphGrabber.position = 0.0
        waitForAButtonPress()
        robot.glyphGrabber.topLeftGlyphGrabber.position = 1.0
        waitForAButtonPress()

        robot.glyphGrabber.topRightGlyphGrabber.position = 0.0
        waitForAButtonPress()
        robot.glyphGrabber.topRightGlyphGrabber.position = 1.0
        waitForAButtonPress()

        robot.glyphGrabber.bottomLeftGlyphGrabber.position = 0.0
        waitForAButtonPress()
        robot.glyphGrabber.bottomLeftGlyphGrabber.position = 1.0
        waitForAButtonPress()

        robot.glyphGrabber.bottomRightGlyphGrabber.position = 0.0
        waitForAButtonPress()
        robot.glyphGrabber.bottomRightGlyphGrabber.position = 1.0
        waitForAButtonPress()
    }

    private fun waitForAButtonPress() {
        while(gamepad1.a && !isStopRequested) {
            idle()
        }
        while(!gamepad1.a && !isStopRequested) {
            idle()
        }
    }
}
