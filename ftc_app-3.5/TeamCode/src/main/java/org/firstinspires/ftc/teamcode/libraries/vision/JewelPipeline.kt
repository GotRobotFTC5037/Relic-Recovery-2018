package org.firstinspires.ftc.teamcode.libraries.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.corningrobotics.enderbots.endercv.OpenCVPipeline
import org.opencv.core.*
import org.opencv.features2d.FeatureDetector
import org.opencv.imgproc.Imgproc
import java.io.File
import java.io.FileWriter
import java.io.IOException

class JewelPipeline : OpenCVPipeline() {

    companion object {
        val TIME_OUT_SECONDS = 2.0

        init {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        }
    }

    enum class JewelPositions {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }

    private val blurOutput = Mat()
    private val hsvOutput = Mat()

    private val bluePositiveThresholdOutput = Mat()
    private val blueNegativeThresholdOutput = Mat()
    private val blueThresholdOutput = Mat()
    private val blueErodeOutput = Mat()
    private val blueDilateOutput = Mat()
    private val findBlueBlobsOutput = MatOfKeyPoint()

    private val redThresholdOutput = Mat()
    private val redErodeOutput = Mat()
    private val redDilateOutput = Mat()
    private val findRedBlobsOutput = MatOfKeyPoint()

    private var lastKnownRedPosition = -1.0
    private var lastKnownBluePosition = -1.0

    val redPosition: Double
        get() {
            val circularity = doubleArrayOf(0.75, 1.0)
            findBlobs(redDilateOutput, 10000.0, circularity, false, findRedBlobsOutput)

            val redBlobs = findRedBlobsOutput.toArray()
            if (redBlobs.isNotEmpty()) {
                lastKnownRedPosition = redBlobs[0].pt.x

            }

            return lastKnownRedPosition
        }

    val bluePosition: Double
        get() {
            val circularity = doubleArrayOf(0.75, 1.0)
            findBlobs(blueDilateOutput, 10000.0, circularity, false, findBlueBlobsOutput)

            val blueBlobs = findBlueBlobsOutput.toArray()
            if (blueBlobs.isNotEmpty()) {
                lastKnownBluePosition = blueBlobs[0].pt.x
            }

            return lastKnownBluePosition
        }

    private val jewelPositions: JewelPositions
        get() {
            val bluePosition = bluePosition
            val redPosition = redPosition

            return if (bluePosition != -1.0 && redPosition != -1.0) {
                if (bluePosition < redPosition) {
                    JewelPositions.BLUE_RED
                } else {
                    JewelPositions.RED_BLUE
                }
            } else {
                JewelPositions.UNKNOWN
            }
        }

    override fun processFrame(rgba: Mat, gray: Mat): Mat {
        val kernelSize = 2 * 10.0 + 1
        Imgproc.blur(rgba, blurOutput, Size(kernelSize, kernelSize))

        Imgproc.cvtColor(blurOutput, hsvOutput, Imgproc.COLOR_BGR2HSV)

        val bluePositiveHueThreshold = doubleArrayOf(0.0, 40.0)
        val blueNegativeHueThreshold = doubleArrayOf(150.0, 180.0)
        val blueSaturationThreshold = doubleArrayOf(150.0, 255.0)
        val blueValueThreshold = doubleArrayOf(100.0, 255.0)
        Core.inRange(hsvOutput, Scalar(bluePositiveHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), Scalar(bluePositiveHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), bluePositiveThresholdOutput)
        Core.inRange(hsvOutput, Scalar(blueNegativeHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), Scalar(blueNegativeHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), blueNegativeThresholdOutput)
        Core.bitwise_or(bluePositiveThresholdOutput, blueNegativeThresholdOutput, blueThresholdOutput)
        Imgproc.erode(blueThresholdOutput, blueErodeOutput, Mat(), Point(-1.0, -1.0), 3, Core.BORDER_CONSTANT, Scalar(-1.0))
        Imgproc.dilate(blueErodeOutput, blueDilateOutput, Mat(), Point(-1.0, -1.0), 15, Core.BORDER_CONSTANT, Scalar(-1.0))

        val redHueThreshold = doubleArrayOf(110.0, 140.0)
        val redSaturationThreshold = doubleArrayOf(230.0, 255.0)
        val redValueThreshold = doubleArrayOf(100.0, 255.0)
        Core.inRange(hsvOutput, Scalar(redHueThreshold[0], redSaturationThreshold[0], redValueThreshold[0]), Scalar(redHueThreshold[1], redSaturationThreshold[1], redValueThreshold[1]), redThresholdOutput)
        Imgproc.erode(redThresholdOutput, redErodeOutput, Mat(), Point(-1.0, -1.0), 3, Core.BORDER_CONSTANT, Scalar(-1.0))
        Imgproc.dilate(redErodeOutput, redDilateOutput, Mat(), Point(-1.0, -1.0), 15, Core.BORDER_CONSTANT, Scalar(-1.0))

        return redDilateOutput
    }

    /**
     * Waits for the jewels to be identified until 5 seconds elapse.
     * @param elapsedTime The elapsed time instance containing the start time of the opmode.
     * @param linearOpMode The instance of LinearOpMode that is running.
     */
    fun waitForJewelIdentification(elapsedTime: ElapsedTime, linearOpMode: LinearOpMode): JewelPositions {
        while(jewelPositions == JewelPositions.UNKNOWN && elapsedTime.seconds() < TIME_OUT_SECONDS) {
            linearOpMode.telemetry.addData("Blue", bluePosition)
            linearOpMode.telemetry.addData("Red", redPosition)
            linearOpMode.telemetry.update()
            linearOpMode.sleep(10)
        }

        return jewelPositions
    }

    private fun findBlobs(input: Mat, minArea: Double, circularity: DoubleArray, darkBlobs: Boolean, blobList: MatOfKeyPoint) {
        val blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB)
        try {
            val tempFile = File.createTempFile("config", ".xml")

            val config = "<?xml version=\"1.0\"?>\n" +
                    "<opencv_storage>\n" +
                    "<thresholdStep>10.</thresholdStep>\n" +
                    "<minThreshold>50.</minThreshold>\n" +
                    "<maxThreshold>220.</maxThreshold>\n" +
                    "<minRepeatability>2</minRepeatability>\n" +
                    "<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n" +
                    "<filterByColor>1</filterByColor>\n" +
                    "<blobColor>" +
                    (if (darkBlobs) 0 else 255) +
                    "</blobColor>\n" +
                    "<filterByArea>1</filterByArea>\n" +
                    "<minArea>" +
                    minArea +
                    "</minArea>\n" +
                    "<maxArea>" +
                    Integer.MAX_VALUE +
                    "</maxArea>\n" +
                    "<filterByCircularity>1</filterByCircularity>\n" +
                    "<minCircularity>" +
                    circularity[0] +
                    "</minCircularity>\n" +
                    "<maxCircularity>" +
                    circularity[1] +
                    "</maxCircularity>\n" +
                    "<filterByInertia>1</filterByInertia>\n" +
                    "<minInertiaRatio>0.1</minInertiaRatio>\n" +
                    "<maxInertiaRatio>" + Integer.MAX_VALUE + "</maxInertiaRatio>\n" +
                    "<filterByConvexity>1</filterByConvexity>\n" +
                    "<minConvexity>0.95</minConvexity>\n" +
                    "<maxConvexity>" + Integer.MAX_VALUE + "</maxConvexity>\n" +
                    "</opencv_storage>\n"

            val writer: FileWriter
            writer = FileWriter(tempFile, false)
            writer.write(config)
            writer.close()
            blobDet.read(tempFile.path)
        } catch (e: IOException) {
            e.printStackTrace()
        }

        blobDet.detect(input, blobList)
    }
}