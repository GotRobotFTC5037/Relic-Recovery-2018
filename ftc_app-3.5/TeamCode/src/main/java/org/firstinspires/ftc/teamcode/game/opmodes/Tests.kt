package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.lib.robot.sensor.RangeSensor

@Autonomous
class CodaDriveTrainStallTest : LinearOpMode() {

    override fun runOpMode() {

        val robot = Coda(this)
        robot.setup()

        val timer = ElapsedTime()

        waitForStart()

        while(opModeIsActive()) {
            val drivePower = timer.milliseconds() / TEST_TIME_SECONDS
            robot.driveTrain.linearDriveAtPower(drivePower)

            if(robot.driveTrain.currentLinearEncoderPosition() > TEST_COMPLETION_THRESHOLD) {
                requestOpModeStop()
            }

            telemetry.addLine("Drive Power: $drivePower")
            telemetry.update()
        }

    }

    companion object {
        private const val TEST_TIME_SECONDS = 50
        private const val TEST_COMPLETION_THRESHOLD = 100
    }

}

@Autonomous
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