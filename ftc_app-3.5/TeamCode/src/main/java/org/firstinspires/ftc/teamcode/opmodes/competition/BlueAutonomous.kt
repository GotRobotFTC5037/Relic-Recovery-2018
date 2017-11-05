package org.firstinspires.ftc.teamcode.opmodes.competition

import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

object BlueAutonomous {

    private lateinit var robot: RelicRecoveryRobot

    fun setup(robot: RelicRecoveryRobot) {
        this.robot = robot
    }

    fun runOpMode() {
        robot.closeGlyphGrabbers()

        robot.linearOpMode.waitForStart()

        robot.lowerJewelStick()
        robot.linearOpMode.sleep(1500)

        val colorSensor = robot.jewelColorSensor
        while(robot.linearOpMode.opModeIsActive()) {
            val red = colorSensor.red()
            val blue = colorSensor.blue()
            if (red > blue) {
                robot.turn(0.5, -15)
                robot.linearOpMode.sleep(1000)
                robot.raiseJewelStick()
                robot.turn(0.5, 15)
                break
            } else if (red < blue) {
                robot.turn(0.5, 15)
                robot.linearOpMode.sleep(1000)
                robot.raiseJewelStick()
                robot.turn(0.5, -15)
                break
            }
        }

        val power = Math.sqrt(2.0)/2
        robot.setDirection(power, power, 0.0)
        while(robot.rangeSensor.cmUltrasonic() >= 35) { robot.linearOpMode.idle() }
        robot.stop()

        robot.setDirection(1.0, 0.0, 0.0)
        robot.linearOpMode.sleep(750)
        robot.setDirection(0.0, 1.0, 0.0)
        robot.linearOpMode.sleep(500)
        robot.stop()
        robot.openGlyphGrabbers()
        robot.setDirection(0.0,-1.0,0.0)
        robot.linearOpMode.sleep(500)
        robot.stop()
    }

}