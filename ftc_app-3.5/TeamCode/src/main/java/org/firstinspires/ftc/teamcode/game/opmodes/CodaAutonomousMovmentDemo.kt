package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.game.robots.Coda
import org.firstinspires.ftc.teamcode.lib.powercontroller.ProportionalPowerController
import org.firstinspires.ftc.teamcode.lib.powercontroller.StaticPowerController

class CodaAutonomousMovmentDemo : LinearOpMode() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = Coda(this)
        robot.driveTrain.turnToHeading(90.0, StaticPowerController(0.5))
        waitForNextStep()
        robot.driveTrain.turnToHeading(180.0, StaticPowerController(0.5))
        waitForNextStep()
        robot.driveTrain.turnToHeading(-90.0, ProportionalPowerController(0.05))
        waitForNextStep()
        robot.driveTrain.turnToHeading(0.0, ProportionalPowerController(0.005))
    }

    private fun waitForNextStep() {
        while (!isStopRequested && !gamepad1.a) {
            sleep(10)
        }
    }

}