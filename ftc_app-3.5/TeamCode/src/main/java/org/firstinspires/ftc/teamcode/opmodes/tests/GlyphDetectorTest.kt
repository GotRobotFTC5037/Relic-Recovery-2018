package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.libraries.vision.GlyphIdentifier

@TeleOp(name = "Glyph Identifier Test")
@Disabled
class GlyphDetectorTest : LinearOpMode() {

    private val glyphDetector = GlyphIdentifier()

    override fun runOpMode() {
        glyphDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())
        waitForStart()
        glyphDetector.enable()

        while (opModeIsActive()) {
            sleep(1000)
        }

        glyphDetector.disable()
    }

}