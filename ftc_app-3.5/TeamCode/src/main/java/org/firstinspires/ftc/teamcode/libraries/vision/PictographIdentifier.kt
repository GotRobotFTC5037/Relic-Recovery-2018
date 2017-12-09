package org.firstinspires.ftc.teamcode.libraries.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.vuforia.CameraDevice
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables


class PictographIdentifier(hardwareMap: HardwareMap) {

    companion object {
        private val VUFORIA_LICENSE_KEY = "ARai2kL/////AAAAGbtywJzFuEENhljmjUYXMOxYSnK" +
                "PHlrkYr0UAA2AiMQwru6hVJxv0+XBwi8d7KGOU90Gku8ubGjJNWdQwPMsvyPOivcjD" +
                "JABZuxgkVr6p8CrwnYIuFTt7v0YDG/9zFztVLkn2jaZnh8p7PkRoPeKFaixcUaDlWz" +
                "U4eydNHAfn0ufnFNm1i5FDsI5pf4UAtwRS2jGHHjyp3/o7sPZpT6Mu1yVT7D6YStzl" +
                "9I/7jMeKHn9lu6zhbnMlOi2D/iY14mOYUvQg1eMiHstkPzUY7IeBngPmCnEjOHWmv7" +
                "XRboufL3JsCTrC8pnnEn5n3pZHei++FW9ovS6Aub89z//Yxq6OhPQ6+WaRNc3VSwFH/KJImw0"

        val TIME_OUT_SECONDS = 10.0
    }

    private val vuforiaLocalizer: ClosableVuforiaLocalizer
    private val relicTrackables: VuforiaTrackables
    private val relicTemplate: VuforiaTrackable

    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        vuforiaLocalizer = ClosableVuforiaLocalizer(parameters)
        this.relicTrackables = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark")
        this.relicTemplate = relicTrackables[0]
    }

    fun activate() {
        CameraDevice.getInstance().setFlashTorchMode(true)
        relicTrackables.activate()
    }

    fun deactivate() {
        relicTrackables.deactivate()
        CameraDevice.getInstance().setFlashTorchMode(false)
        vuforiaLocalizer.close()
    }

    private val identifiedPictograph: RelicRecoveryVuMark
        get() = RelicRecoveryVuMark.from(relicTemplate)

    fun waitForPictographIdentification(elapsedTime: ElapsedTime, linearOpMode: LinearOpMode): RelicRecoveryVuMark {
        while(identifiedPictograph == RelicRecoveryVuMark.UNKNOWN && elapsedTime.seconds() < TIME_OUT_SECONDS) {
            linearOpMode.sleep(100)
        }

        return identifiedPictograph
    }

}