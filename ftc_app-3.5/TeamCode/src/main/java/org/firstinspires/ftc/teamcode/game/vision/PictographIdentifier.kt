package org.firstinspires.ftc.teamcode.game.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import com.vuforia.CameraDevice
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables
import org.firstinspires.ftc.teamcode.game.elements.CryptoBox
import org.firstinspires.ftc.teamcode.lib.vision.ClosableVuforiaLocalizer

/**
 * A class that is used to identify the pictograph images on the side wall in
 * the 2017-2018 FTC game.
 */
class PictographIdentifier(private var linearOpMode: LinearOpMode) {

    private var vuforiaLocalizer: ClosableVuforiaLocalizer
    private var relicTrackables: VuforiaTrackables
    private var relicTemplate: VuforiaTrackable

    init {
        val cameraMonitorViewId = linearOpMode.hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId", "id",
            linearOpMode.hardwareMap.appContext.packageName
        )

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
    fun waitForPictographIdentification(): CryptoBox.ColumnPosition? {

        if (!linearOpMode.isStopRequested) {
            linearOpMode.telemetry.log().add("Waiting for pictograph identification.")

            val elapsedTime = ElapsedTime()
            while (elapsedTime.milliseconds() < TIME_OUT_DURATION && !linearOpMode.isStopRequested) {
                val pictograph = this.identifiedPictograph

                if (elapsedTime.milliseconds() >= 1000) {
                    CameraDevice.getInstance().setFlashTorchMode(true)
                }

                if (pictograph != RelicRecoveryVuMark.UNKNOWN) {
                    return when (pictograph) {
                        RelicRecoveryVuMark.LEFT -> {
                            linearOpMode.telemetry.log().add("Left Pictograph identified.")
                            CryptoBox.ColumnPosition.LEFT
                        }

                        RelicRecoveryVuMark.CENTER -> {
                            linearOpMode.telemetry.log().add("Center Pictograph identified")
                            CryptoBox.ColumnPosition.CENTER
                        }

                        RelicRecoveryVuMark.RIGHT -> {
                            linearOpMode.telemetry.log().add("Right Pictograph identified.")
                            CryptoBox.ColumnPosition.RIGHT
                        }

                        else -> {
                            // This should never happen.
                            null
                        }
                    }
                }

                linearOpMode.sleep(10)
            }

            linearOpMode.telemetry.log().add("Failed to identify the pictograph.")
        }

        return null
    }

    companion object {
        private const val VUFORIA_LICENSE_KEY = "ARai2kL/////AAAAGbtywJzFuEENhljmjUYXMOxYSnK" +
                "PHlrkYr0UAA2AiMQwru6hVJxv0+XBwi8d7KGOU90Gku8ubGjJNWdQwPMsvyPOivcjD" +
                "JABZuxgkVr6p8CrwnYIuFTt7v0YDG/9zFztVLkn2jaZnh8p7PkRoPeKFaixcUaDlWz" +
                "U4eydNHAfn0ufnFNm1i5FDsI5pf4UAtwRS2jGHHjyp3/o7sPZpT6Mu1yVT7D6YStzl" +
                "9I/7jMeKHn9lu6zhbnMlOi2D/iY14mOYUvQg1eMiHstkPzUY7IeBngPmCnEjOHWmv7" +
                "XRboufL3JsCTrC8pnnEn5n3pZHei++FW9ovS6Aub89z//Yxq6OhPQ6+WaRNc3VSwFH/KJImw0"

        const val TIME_OUT_DURATION = 2000
    }

}