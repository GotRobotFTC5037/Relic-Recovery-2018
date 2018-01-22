package org.firstinspires.ftc.teamcode.game.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.game.components.RelicGrabber
import org.firstinspires.ftc.teamcode.lib.robot.Robot

/**
 * Created by Andrew on 1/19/18.
 */
class TestBedBot(linearOpMode: LinearOpMode): Robot(linearOpMode) {

    companion object {
        private const val RELIC_GRABBER = "relic_grabber"
    }

    fun setup() {
        this.addComponent(RelicGrabber(linearOpMode), RELIC_GRABBER)
    }

    val relicGrabber: RelicGrabber
        get() = components[RELIC_GRABBER] as RelicGrabber
}