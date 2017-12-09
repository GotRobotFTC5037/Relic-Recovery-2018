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

class JewelConfigurationDetector : OpenCVPipeline() {

    companion object {
        val TIME_OUT_SECONDS = 5.0

        init {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        }
    }

    enum class JewelConfiguration {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }

    private val blurOutput = Mat()
    private val hslOutput = Mat()
    private val hsvOutput = Mat()

    private val whiteThresholdOutput = Mat()
    private val whiteFindContoursOutput = mutableListOf<MatOfPoint>()

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

    var shouldShowDetectedJewelPositions = true

    /**
     * Waits for the jewels to be identified until 5 seconds elapse.
     * @param elapsedTime The elapsed time instance containing the start time of the opmode.
     * @param linearOpMode The instance of LinearOpMode that is running.
     */
    fun waitForJewelIdentification(elapsedTime: ElapsedTime, linearOpMode: LinearOpMode): JewelConfiguration {

        while (elapsedTime.seconds() < TIME_OUT_SECONDS && !linearOpMode.isStopRequested) {
            val jewelConfiguration = getJewelConfiguration()

            if (jewelConfiguration != JewelConfiguration.UNKNOWN) {
                return jewelConfiguration
            }

            linearOpMode.sleep(10)
        }

        return JewelConfiguration.UNKNOWN
    }

    /**
     * Finds the blobs of decently sized, red, circular objects in the image.
     * @return The KeyPoint of the first blob identified in the image.
     */
    @Synchronized
    fun getRedJewelKeyPoint(): KeyPoint? {
        findBlobs(redDilateOutput, 10000.0, doubleArrayOf(0.75, 1.0), false, findRedBlobsOutput)
        val redBlobs = findRedBlobsOutput.toArray()
        return if (redBlobs.isNotEmpty()) {
            redBlobs[0]
        } else {
            null
        }
    }

    /**
     * Finds the blobs of decently sized, blue, circular objects in the image.
     * @return The KeyPoint of the first blob identified in the image.
     */
    @Synchronized
    fun getBlueJewelKeyPoint(): KeyPoint? {
        findBlobs(blueDilateOutput, 10000.0, doubleArrayOf(0.75, 1.0), false, findBlueBlobsOutput)
        val blueBlobs = findBlueBlobsOutput.toArray()
        return if (blueBlobs.isNotEmpty()) {
            blueBlobs[0]
        } else {
            null
        }
    }

    /**
     * Finds the location of the white line.
     * @return A Point of the location of the white line.
     */
    @Synchronized
    fun getWhiteLinePoint(): Point? {
        Imgproc.findContours(whiteThresholdOutput, whiteFindContoursOutput, Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE)
        return if (whiteFindContoursOutput.isNotEmpty()) {
            val whiteContours = whiteFindContoursOutput[0].toArray()
            if (whiteContours.isNotEmpty()) {
                val contour = whiteContours[0]
                Point(contour.x, contour.y)
            } else {
                null
            }
        } else {
            null
        }

    }

    /**
     * Determines the configuration of the jewels in the image.
     * @return The configuration of the jewels in the image.
     */
    @Synchronized private fun getJewelConfiguration(): JewelConfiguration {
        val bluePosition = getBlueJewelKeyPoint()
        val redPosition = getRedJewelKeyPoint()
        val whiteLinePosition = getWhiteLinePoint()

        return when {
            bluePosition != null && redPosition != null ->
                when {
                    bluePosition.pt.x < redPosition.pt.x -> JewelConfiguration.BLUE_RED
                    bluePosition.pt.x > redPosition.pt.x -> JewelConfiguration.RED_BLUE
                    else -> JewelConfiguration.UNKNOWN
                }


            whiteLinePosition != null && bluePosition != null ->
                when {
                    bluePosition.pt.x < whiteLinePosition.x -> JewelConfiguration.BLUE_RED
                    bluePosition.pt.x > whiteLinePosition.x -> JewelConfiguration.RED_BLUE
                    else -> JewelConfiguration.UNKNOWN
                }

            whiteLinePosition != null && redPosition != null ->
                when {
                    redPosition.pt.x < whiteLinePosition.x -> JewelConfiguration.RED_BLUE
                    redPosition.pt.x > whiteLinePosition.x -> JewelConfiguration.BLUE_RED
                    else -> JewelConfiguration.UNKNOWN
                }

            else -> JewelConfiguration.UNKNOWN
        }
    }

    /**
     * Processes the imputed frame for jewels
     * @return An image to show on the screen that shows the identified jewels.
     */
    override fun processFrame(rgba: Mat, gray: Mat): Mat {

        // Blur the image to get rid of noise.
        Imgproc.blur(rgba, blurOutput, Size(21.0, 21.0))
        Imgproc.cvtColor(blurOutput, hsvOutput, Imgproc.COLOR_BGR2HSV)
        Imgproc.cvtColor(blurOutput, hslOutput, Imgproc.COLOR_BGR2HLS)

        // Filter out anything that is not the color of the white line in between the jewels.
        val whiteHueThreshold = doubleArrayOf(0.0, 180.0)
        val whiteSaturationThreshold = doubleArrayOf(0.0, 255.0)
        val whiteLuminanceThreshold = doubleArrayOf(25.0, 255.0)
        Core.inRange(hsvOutput, Scalar(whiteHueThreshold[0], whiteSaturationThreshold[0], whiteLuminanceThreshold[0]), Scalar(whiteHueThreshold[1], whiteSaturationThreshold[1], whiteLuminanceThreshold[1]), whiteThresholdOutput)

        // Filter out anything that is not the color of the blue jewel.
        val bluePositiveHueThreshold = doubleArrayOf(0.0, 40.0)
        val blueNegativeHueThreshold = doubleArrayOf(150.0, 180.0)
        val blueSaturationThreshold = doubleArrayOf(60.0, 255.0)
        val blueValueThreshold = doubleArrayOf(25.0, 255.0)
        Core.inRange(hsvOutput, Scalar(bluePositiveHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), Scalar(bluePositiveHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), bluePositiveThresholdOutput)
        Core.inRange(hsvOutput, Scalar(blueNegativeHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), Scalar(blueNegativeHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), blueNegativeThresholdOutput)
        Core.bitwise_or(bluePositiveThresholdOutput, blueNegativeThresholdOutput, blueThresholdOutput)
        Imgproc.erode(blueThresholdOutput, blueErodeOutput, Mat(), Point(-1.0, -1.0), 3, Core.BORDER_CONSTANT, Scalar(-1.0))
        Imgproc.dilate(blueErodeOutput, blueDilateOutput, Mat(), Point(-1.0, -1.0), 15, Core.BORDER_CONSTANT, Scalar(-1.0))

        // Filter out anything that is not the color of the red jewel.
        val redHueThreshold = doubleArrayOf(110.0, 140.0)
        val redSaturationThreshold = doubleArrayOf(150.0, 255.0)
        val redValueThreshold = doubleArrayOf(25.0, 255.0)
        Core.inRange(hsvOutput, Scalar(redHueThreshold[0], redSaturationThreshold[0], redValueThreshold[0]), Scalar(redHueThreshold[1], redSaturationThreshold[1], redValueThreshold[1]), redThresholdOutput)
        Imgproc.erode(redThresholdOutput, redErodeOutput, Mat(), Point(-1.0, -1.0), 3, Core.BORDER_CONSTANT, Scalar(-1.0))
        Imgproc.dilate(redErodeOutput, redDilateOutput, Mat(), Point(-1.0, -1.0), 15, Core.BORDER_CONSTANT, Scalar(-1.0))

        // Show the detected jewel positions on screen if requested.
        if (shouldShowDetectedJewelPositions) {

            // Identify the red and blue jewels.
            val redKeyPoint = this.getRedJewelKeyPoint()
            val blueKeyPoint = this.getBlueJewelKeyPoint()
            val whiteLinePoint = this.getWhiteLinePoint()

            // Clearly identify the white line on the screen.
            if (whiteLinePoint != null) {
                val x = whiteLinePoint.x
                val y = whiteLinePoint.y
                val yellowScalar = Scalar(0.0, 255.0, 255.0)
                Imgproc.circle(rgba, Point(x, y), 5, yellowScalar, 3)
                Imgproc.putText(rgba, "Line", Point(x, y - 20), 0, 0.8, yellowScalar, 2)
            }

            // Clearly identify the red jewel on the screen.
            if (redKeyPoint != null) {
                val x = redKeyPoint.pt.x
                val y = redKeyPoint.pt.y
                val size = redKeyPoint.size
                val redScalar = Scalar(255.0, 0.0, 0.0)
                Imgproc.rectangle(rgba, Point(x, y), Point(x + size, y + size), redScalar, 3)
                Imgproc.putText(rgba, "Red Jewel", Point(x, y - 20), 0, 0.8, redScalar, 3)
            }

            // Clearly identify the blue jewel on the screen.
            if (blueKeyPoint != null) {
                val x = blueKeyPoint.pt.x
                val y = blueKeyPoint.pt.y
                val size = blueKeyPoint.size
                val blueScalar = Scalar(0.0, 0.0, 255.0)
                Imgproc.rectangle(rgba, Point(x, y), Point(x + size, y + size), blueScalar, 3)
                Imgproc.putText(rgba, "Blue Jewel", Point(x, y - 20), 0, 0.8, blueScalar, 2)
            }

        }

        return rgba
    }

    /**
     * Finds the blobs in an image. Usually used for finding the jewels in autonomous.
     * @param input The input Mat to find blobs in.
     * @param minArea The minimum area required to recognize as a blob.
     * @param circularity The circularity required to recognize as a blob.
     * @param darkBlobs Informs the blob detector if the recognition should be based on light blobs or dark blobs.
     * @param blobList The MatOfKeyPoints to use as the output for recognition.
     */
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