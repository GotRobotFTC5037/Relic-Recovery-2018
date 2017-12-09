package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector

@TeleOp(name = "OpenCV Test")
class OpenCVTest : LinearOpMode() {

    private val jewelDetector = JewelConfigurationDetector()

    override fun runOpMode() {
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())
        waitForStart()
        jewelDetector.enable()

        while (opModeIsActive()) {
            telemetry.addData("Red", jewelDetector.getBlueJewelKeyPoint())
            telemetry.addData("Blue", jewelDetector.getRedJewelKeyPoint())
            telemetry.update()
        }

        jewelDetector.disable()
    }

}