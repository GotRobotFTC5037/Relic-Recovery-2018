package org.firstinspires.ftc.teamcode.libraries.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.vuforia.CameraDevice
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables

/**
 * A class that is used to identify the pictograph images on the side wall in
 * the 2017-2018 FTC game.
 */
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

    /**
     * Sets up the vuforia localizer and the vuforia trackables for usage.
     */
    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        vuforiaLocalizer = ClosableVuforiaLocalizer(parameters)
        this.relicTrackables = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark")
        this.relicTemplate = relicTrackables[0]
    }

    /**
     * Activates vuforia vumark tracking and turns on the flash.
     */
    fun activate() {
        CameraDevice.getInstance().setFlashTorchMode(true)
        relicTrackables.activate()
    }

    /**
     * Deactivates vuforia vumark tracking, turns off the flash and closes the vuforia localizer.
     */
    fun deactivate() {
        relicTrackables.deactivate()
        CameraDevice.getInstance().setFlashTorchMode(false)
        vuforiaLocalizer.close()
    }

    private val identifiedPictograph: RelicRecoveryVuMark
        get() = RelicRecoveryVuMark.from(relicTemplate)

    /**
     * Waits until the pictograph is identified or it has been a specified number of seconds in an ElapsedTime.
     * @return The identified pictograph.
     */
    fun waitForPictographIdentification(elapsedTime: ElapsedTime, linearOpMode: LinearOpMode): RelicRecoveryVuMark {

        while (elapsedTime.seconds() < TIME_OUT_SECONDS && !linearOpMode.isStopRequested) {
            val pictograph = this.identifiedPictograph

            if (pictograph != RelicRecoveryVuMark.UNKNOWN) {
                return pictograph
            }

            linearOpMode.sleep(100)
        }

        return RelicRecoveryVuMark.UNKNOWN
    }

}