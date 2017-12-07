package org.firstinspires.ftc.teamcode.libraries.vision

import android.graphics.Bitmap
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import com.vuforia.Image
import com.vuforia.PIXEL_FORMAT
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.opencv.android.Utils
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc


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

    private val vuforiaLocalizer: VuforiaLocalizer
    private val relicTrackables: VuforiaTrackables
    private val relicTemplate: VuforiaTrackable

    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)
        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK

        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters)
        this.relicTrackables = vuforiaLocalizer.loadTrackablesFromAsset("RelicVuMark")
        this.relicTemplate = relicTrackables[0]
    }

    fun activate() {
        relicTrackables.activate()
    }

    fun deactivate() {
        relicTrackables.deactivate()
    }

    private fun getIdentifiedPictograph(): RelicRecoveryVuMark = RelicRecoveryVuMark.from(relicTemplate)

    fun waitForPictographIdentification(elapsedTime: ElapsedTime, linearOpMode: LinearOpMode): RelicRecoveryVuMark {
        while(getIdentifiedPictograph() == RelicRecoveryVuMark.UNKNOWN && elapsedTime.seconds() < TIME_OUT_SECONDS) {
            linearOpMode.sleep(100)
        }

        return getIdentifiedPictograph()
    }

    fun readFrame(): Mat? {
        val frame: VuforiaLocalizer.CloseableFrame

        try {
            frame = vuforiaLocalizer.frameQueue.take()
        } catch (e: InterruptedException) {
            e.printStackTrace()
            return null
        }

        val numImages = frame.numImages

        val rgb: Image = (0 until numImages)
                .firstOrNull { frame.getImage(it.toInt()).format == PIXEL_FORMAT.RGB565 }
                ?.let { frame.getImage(it.toInt()) } ?: return null

        val bm = Bitmap.createBitmap(rgb.width, rgb.height, Bitmap.Config.RGB_565)
        bm.copyPixelsFromBuffer(rgb.pixels)

        val mat = Mat(bm.width, bm.height, CvType.CV_8UC4)
        Utils.bitmapToMat(bm, mat)

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR)
        frame.close()
        return mat
    }

    fun getSetupAngleAdjustment(): Double {
        val pose = (relicTemplate.listener as VuforiaTrackableDefaultListener).pose

        if (pose != null) {
            val trans = pose.translation
            val rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)

            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            // val tX = trans.get(0).toDouble()
            // val tY = trans.get(1).toDouble()
            // val tZ = trans.get(2).toDouble()

            // Extract the rotational components of the target relative to the robot
            val rX = rot.firstAngle.toDouble()
            val rY = rot.secondAngle.toDouble()
            val rZ = rot.thirdAngle.toDouble()

            return rX
        } else {
            return 0.0
        }
    }

}