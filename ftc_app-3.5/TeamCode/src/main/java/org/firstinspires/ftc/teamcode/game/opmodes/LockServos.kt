package org.firstinspires.ftc.teamcode.game.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.game.components.GlyphGrabber
import org.firstinspires.ftc.teamcode.game.robots.Coda

@TeleOp
class LockServos: LinearOpMode() {
    override fun runOpMode() {
        val robot = Coda(this)
        robot.setup()
        waitForStart()
        robot.glyphGrabber.setState(GlyphGrabber.GlyphGrabberState.OPEN)
        while (opModeIsActive()) { idle() }
    }
}