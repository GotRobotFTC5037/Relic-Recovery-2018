package org.firstinspires.ftc.teamcode.opmodes.tests

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.corningrobotics.enderbots.endercv.CameraViewDisplay
import org.firstinspires.ftc.teamcode.libraries.vision.JewelConfigurationDetector

@TeleOp(name = "Jewel Detector Test")
@Disabled
class JewelDetectorTest : LinearOpMode() {

    private val jewelDetector = JewelConfigurationDetector()

    override fun runOpMode() {
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance())
        waitForStart()
        jewelDetector.enable()

        while (opModeIsActive()) {
            val redJewelKeyPoint = jewelDetector.getRedJewelKeyPoint()
            val blueJewelKeyPoint = jewelDetector.getBlueJewelKeyPoint()
            val whiteJewelPoint = jewelDetector.getWhiteLinePoint()

            if (redJewelKeyPoint != null) {
                telemetry.addData("Red", redJewelKeyPoint.pt.x)
            } else {
                telemetry.addData("Red", "Unknown")
            }

            if (blueJewelKeyPoint != null) {
                telemetry.addData("Blue", blueJewelKeyPoint.pt.x)
            } else {
                telemetry.addData("Blue", "Unknown")
            }

            if (whiteJewelPoint != null) {
                telemetry.addData("Line", whiteJewelPoint.x)
            } else {
                telemetry.addData("Line", "Unknown")
            }

            telemetry.update()
        }

        jewelDetector.disable()
    }

}