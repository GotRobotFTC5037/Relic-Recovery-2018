package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.libraries.vision.JewelPipeline

@TeleOp(name = "OpenCV Test")
class OpenCVTest : LinearOpMode() {

    private val jewelDetector = JewelPipeline()

    override fun runOpMode() {
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())
        waitForStart()
        jewelDetector.enable()

        while (opModeIsActive()) {
            telemetry.addLine("Blue Jewel Position:")

            /**
            val jewelConfiguration = jewelDetector.jewelConfiguration
            when (jewelConfiguration) {
                JewelPipeline.JewelConfiguration.BLUE_RED -> telemetry.addLine("Left")
                JewelPipeline.JewelConfiguration.RED_BLUE -> telemetry.addLine("Right")
                JewelPipeline.JewelConfiguration.UNKNOWN -> telemetry.addLine("Unknown")
                else -> {}
            }
            */

            telemetry.update()
        }

        jewelDetector.disable()
    }

}