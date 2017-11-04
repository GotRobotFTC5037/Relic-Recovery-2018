package org.firstinspires.ftc.teamcode.opmodes.competition

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robots.RelicRecoveryRobot

@Autonomous(name = "Blue Autonomous")
class BlueAuto : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = RelicRecoveryRobot()
        robot.setup(this)

        waitForStart()

        robot.lowerJewelStick()
        sleep(1500)

        val colorSensor = robot.colorSensor
        while(opModeIsActive()) {
            if (colorSensor.red() > colorSensor.blue()) {
                robot.turn(0.5, -15)
                sleep(1000)
                robot.raiseJewelStick()
                robot.turn(0.5, 15)
                break
            } else if (colorSensor.red() < colorSensor.blue()) {
                robot.turn(0.5, 15)
                sleep(1000)
                robot.raiseJewelStick()
                robot.turn(0.5, -15)
                break
            }
        }

        val power = Math.sqrt(2.0)/2
        robot.setDirection(power, power, 0.0)
        sleep(2500)

        robot.stop()
    }

}
