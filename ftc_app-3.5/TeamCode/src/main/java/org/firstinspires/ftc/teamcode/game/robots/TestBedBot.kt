package org.firstinspires.ftc.teamcode.game.robots

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.game.components.RelicGrabber
import org.firstinspires.ftc.teamcode.libraries.robot.Robot

/**
 * Created by Andrew on 1/19/18.
 */
class TestBedBot(linearOpMode: LinearOpMode): Robot(linearOpMode) {
    fun setup() {
        this.addComponent(RelicGrabber(linearOpMode), "QuarterNote")
    }
    val relicGrabber: RelicGrabber
    get() = attachments["QuarterNote"] as RelicGrabber
}