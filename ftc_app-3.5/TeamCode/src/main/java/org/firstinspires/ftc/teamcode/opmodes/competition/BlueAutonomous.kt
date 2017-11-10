package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Blue Autonomous")
class BlueAutonomous: LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot

        if(robot.isSetup) {
            robot.linearOpMode = this
        } else {
            robot.setup(this)
        }

        waitForStart()
        robot.closeGlyphGrabbers()
        robot.waitForGyroCalibration()

        /*
        robot.lowerJewelStick()
        sleep(1500)

        val colorSensor = robot.jewelColorSensor
        while(opModeIsActive()) {
            val red = colorSensor.red()
            val blue = colorSensor.blue()
            if (red > blue) {
                robot.turn(0.5, -15)
                sleep(1000)
                robot.raiseJewelStick()
                sleep(2000)
                robot.turn(0.5, 15)
                break
            } else if (red < blue) {
                robot.turn(0.5, 15)
                sleep(1000)
                robot.raiseJewelStick()
                sleep(2000)
                robot.turn(0.5, -15)
                break
            }
        }
        */

        robot.driveUntilThresholdReached(100.0, 0.5)
        robot.setStrafePower(0.75)
        sleep(750)
        robot.stopAllDriveMotors()
    }


}