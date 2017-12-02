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
            telemetry.addData("Red", jewelDetector.bluePosition)
            telemetry.addData("Blue", jewelDetector.redPosition)
            telemetry.update()
        }

        jewelDetector.disable()
    }

}