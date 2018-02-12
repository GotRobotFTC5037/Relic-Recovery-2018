package org.firstinspires.ftc.teamcode.game.vision

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.corningrobotics.enderbots.endercv.OpenCVPipeline
import org.opencv.core.*
import org.opencv.features2d.FeatureDetector
import org.opencv.imgproc.Imgproc
import java.io.File
import java.io.FileWriter
import java.io.IOException

class JewelConfigurationDetector(private val linearOpMode: LinearOpMode) : OpenCVPipeline() {

    private val resizeOutput = Mat()
    private val hsvOutput = Mat()

    private val extractGreenChannelOutput = Mat()
    private val greenNormalizationOutput = Mat()

    private val extractRedChannelOutput = Mat()
    private val redNormalizationOutput = Mat()
    private val redMinusBlueOutput = Mat()
    private val normalizedRedMinusBlueOutput = Mat()
    private val redMinusGreenOutput = Mat()
    private val normalizedRedMinusGreenOutput = Mat()
    private val thresholdRedChannelOutput = Mat()
    private val redErodeOutput = Mat()
    private val redDilateOutput = Mat()
    private val redFindBlobsInput = Mat()
    private val findRedBlobsOutput = MatOfKeyPoint()

    private val extractBlueChannelOutput = Mat()
    private val blueNormalizationOutput = Mat()
    private val blueMinusRedOutput = Mat()
    private val normalizedBlueMinusRedOutput = Mat()
    private val blueMinusGreenOutput = Mat()
    private val normalizedBlueMinusGreenOutput = Mat()
    private val thresholdBlueChannelOutput = Mat()
    private val blueErodeOutput = Mat()
    private val blueDilateOutput = Mat()
    private val blueFindBlobsInput = Mat()
    private val findBlueBlobsOutput = MatOfKeyPoint()

    private val whiteThresholdOutput = Mat()
    private val whiteErodeOutput = Mat()

    private val detectedObjectsOutput = Mat()

    /**
     * Waits for the jewels to be identified.
     */
    fun waitForJewelIdentification(): JewelConfiguration {

        linearOpMode.telemetry.log().add("Waiting for jewel configuration identification.")
        val elapsedTime = ElapsedTime()
        while (elapsedTime.milliseconds() < TIME_OUT_DURATION && !linearOpMode.isStopRequested) {
            val jewelConfiguration = getJewelConfiguration()

            if (jewelConfiguration != JewelConfiguration.UNKNOWN) {
                when (jewelConfiguration) {
                    JewelConfiguration.RED_BLUE ->
                        linearOpMode.telemetry.log()
                            .add("Red-Blue configuration identified at ${elapsedTime.milliseconds()} milliseconds")
                    JewelConfiguration.BLUE_RED ->
                        linearOpMode.telemetry.log()
                            .add("Blue-Red configuration identified at ${elapsedTime.milliseconds()} milliseconds")

                    else -> { /* This should never happen */
                    }
                }
                return jewelConfiguration
            }

            linearOpMode.sleep(10)
        }

        linearOpMode.telemetry.log().add("Failed to identify jewel configuration.")
        return JewelConfiguration.UNKNOWN
    }

    /**
     * Finds the blobs of decently sized, red, circular objects in the image.
     * @return The KeyPoint of the first blob identified in the image.
     */
    @Synchronized
    private fun getRedJewelKeyPoint(): KeyPoint? {
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
    private fun getBlueJewelKeyPoint(): KeyPoint? {
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
    private fun getWhiteLinePoint(): Point? {
        val findContoursOutput = ArrayList<MatOfPoint>()
        val filterContoursOutput = ArrayList<MatOfPoint>()

        Imgproc.findContours(
            whiteErodeOutput,
            findContoursOutput,
            Mat(),
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        filterContours(
            findContoursOutput,
            0.0, 0.0,
            10.0, 40.0,
            0.0, 1000.0,
            doubleArrayOf(0.0, 100.0),
            1000.0, 0.0,
            0.0, 1000.0,
            filterContoursOutput
        )

        return if (filterContoursOutput.isNotEmpty()) {
            val boundingRectangle = Imgproc.boundingRect(filterContoursOutput[0])
            Point(
                boundingRectangle.x.toDouble() + (boundingRectangle.width / 2),
                boundingRectangle.y.toDouble() + (boundingRectangle.height / 2)
            )
        } else {
            null
        }

    }

    enum class JewelConfiguration {
        RED_BLUE,
        BLUE_RED,
        UNKNOWN
    }

    /**
     * Determines the configuration of the jewels in the image.
     * @return The configuration of the jewels in the image.
     */
    @Synchronized
    private fun getJewelConfiguration(): JewelConfiguration {
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

        // Resize the image for faster processing.
        val originalSize = rgba.size()
        Imgproc.resize(rgba, resizeOutput, Size(320.0, 180.0), 0.0, 0.0, Imgproc.INTER_AREA)

        // Extract each color channel.
        Core.extractChannel(resizeOutput, extractRedChannelOutput, 0)
        Core.extractChannel(resizeOutput, extractGreenChannelOutput, 1)
        Core.extractChannel(resizeOutput, extractBlueChannelOutput, 2)
        Core.normalize(
            extractRedChannelOutput,
            redNormalizationOutput,
            0.0,
            255.0,
            Core.NORM_MINMAX
        )
        Core.normalize(
            extractGreenChannelOutput,
            greenNormalizationOutput,
            0.0,
            130.0,
            Core.NORM_MINMAX
        )
        Core.normalize(
            extractBlueChannelOutput,
            blueNormalizationOutput,
            0.0,
            255.0,
            Core.NORM_MINMAX
        )

        // Find the red jewel.
        Core.subtract(redNormalizationOutput, blueNormalizationOutput, redMinusBlueOutput)
        Core.normalize(
            redMinusBlueOutput,
            normalizedRedMinusBlueOutput,
            0.0,
            255.0,
            Core.NORM_MINMAX
        )
        Core.subtract(normalizedRedMinusBlueOutput, greenNormalizationOutput, redMinusGreenOutput)
        Core.normalize(
            redMinusGreenOutput,
            normalizedRedMinusGreenOutput,
            0.0,
            255.0,
            Core.NORM_MINMAX
        )
        Imgproc.threshold(
            normalizedRedMinusGreenOutput,
            thresholdRedChannelOutput,
            125.0,
            255.0,
            Imgproc.THRESH_BINARY
        )
        Imgproc.erode(
            thresholdRedChannelOutput,
            redErodeOutput,
            Mat(),
            Point(-1.0, -1.0),
            1,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )
        Imgproc.dilate(
            redErodeOutput,
            redDilateOutput,
            Mat(),
            Point(-1.0, -1.0),
            6,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )
        Imgproc.erode(
            redDilateOutput,
            redFindBlobsInput,
            Mat(),
            Point(-1.0, -1.0),
            5,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )
        findBlobs(
            redFindBlobsInput,
            600.0,
            doubleArrayOf(0.0, 100.0),
            false,
            findRedBlobsOutput
        )

        // Find the blue jewel.
        Core.subtract(blueNormalizationOutput, redNormalizationOutput, blueMinusRedOutput)
        Core.normalize(
            blueMinusRedOutput,
            normalizedBlueMinusRedOutput,
            0.0,
            255.0,
            Core.NORM_MINMAX
        )
        Core.subtract(normalizedBlueMinusRedOutput, greenNormalizationOutput, blueMinusGreenOutput)
        Core.normalize(
            blueMinusGreenOutput,
            normalizedBlueMinusGreenOutput,
            0.0,
            255.0,
            Core.NORM_MINMAX
        )
        Imgproc.threshold(
            normalizedBlueMinusGreenOutput,
            thresholdBlueChannelOutput,
            10.0,
            255.0,
            Imgproc.THRESH_BINARY
        )
        Imgproc.erode(
            thresholdBlueChannelOutput,
            blueErodeOutput,
            Mat(),
            Point(-1.0, -1.0),
            1,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )
        Imgproc.dilate(
            blueErodeOutput,
            blueDilateOutput,
            Mat(),
            Point(-1.0, -1.0),
            6,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )
        Imgproc.erode(
            blueDilateOutput,
            blueFindBlobsInput,
            Mat(),
            Point(-1.0, -1.0),
            5,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )
        findBlobs(
            blueFindBlobsInput,
            600.0,
            doubleArrayOf(0.0, 100.0),
            false,
            findBlueBlobsOutput
        )

        // Find the white line.
        Imgproc.cvtColor(resizeOutput, hsvOutput, Imgproc.COLOR_BGR2HSV)
        Core.inRange(
            hsvOutput,
            Scalar(0.0, 0.0, 205.0),
            Scalar(180.0, 75.0, 255.0),
            whiteThresholdOutput
        )
        Imgproc.erode(
            whiteThresholdOutput,
            whiteErodeOutput,
            Mat(),
            Point(-1.0, -1.0),
            1,
            Core.BORDER_CONSTANT,
            Scalar(-1.0)
        )

        // Show all detected elements on the screen.
        resizeOutput.copyTo(detectedObjectsOutput)

        // Identify the red and blue jewels.
        val redKeyPoint = this.getRedJewelKeyPoint()
        val blueKeyPoint = this.getBlueJewelKeyPoint()
        val whiteLinePoint = this.getWhiteLinePoint()

        // Show the identified red jewel on the screen.
        if (redKeyPoint != null) {
            val x = redKeyPoint.pt.x
            val y = redKeyPoint.pt.y
            val size = redKeyPoint.size
            val radius = size / 2
            val redScalar = Scalar(255.0, 0.0, 0.0)
            Imgproc.rectangle(
                detectedObjectsOutput,
                Point(x - radius, y - radius),
                Point(x + radius, y + radius),
                redScalar,
                3
            )
            Imgproc.putText(
                detectedObjectsOutput,
                "Red",
                Point(x - radius, y - 5 - radius),
                0,
                0.5,
                redScalar,
                2
            )
        }

        // Show the identified blue jewel on the screen.
        if (blueKeyPoint != null) {
            val x = blueKeyPoint.pt.x
            val y = blueKeyPoint.pt.y
            val size = blueKeyPoint.size
            val radius = size / 2
            val blueScalar = Scalar(0.0, 0.0, 255.0)
            Imgproc.rectangle(
                detectedObjectsOutput,
                Point(x - radius, y - radius),
                Point(x + radius, y + radius),
                blueScalar,
                3
            )
            Imgproc.putText(
                detectedObjectsOutput,
                "Blue",
                Point(x - radius, y - 5 - radius),
                0,
                0.5,
                blueScalar,
                2
            )
        }

        // Show the identified white line on the screen.
        if (whiteLinePoint != null) {
            val x = whiteLinePoint.x
            val y = whiteLinePoint.y
            val yellowScalar = Scalar(255.0, 255.0, 0.0)
            Imgproc.circle(detectedObjectsOutput, Point(x, y), 5, yellowScalar, 5)
        }

        Imgproc.resize(
            detectedObjectsOutput,
            detectedObjectsOutput,
            originalSize,
            0.0,
            0.0,
            Imgproc.INTER_AREA
        )

        return detectedObjectsOutput
    }

    /**
     * Finds the blobs in an image. Usually used for finding the jewels in autonomous.
     * @param input The input Mat to find blobs in.
     * @param minArea The minimum area required to recognize as a blob.
     * @param circularity The circularity required to recognize as a blob.
     * @param darkBlobs Informs the blob detector if the recognition should be based on light blobs or dark blobs.
     * @param blobList The MatOfKeyPoints to use as the sumOutput for recognition.
     */
    private fun findBlobs(
        input: Mat,
        minArea: Double,
        circularity: DoubleArray,
        darkBlobs: Boolean,
        blobList: MatOfKeyPoint
    ) {
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

    private fun filterContours(
        inputContours: List<MatOfPoint>,
        minArea: Double, minPerimeter: Double,
        minWidth: Double, maxWidth: Double,
        minHeight: Double, maxHeight: Double,
        solidity: DoubleArray,
        maxVertexCount: Double, minVertexCount: Double,
        minRatio: Double, maxRatio: Double,
        output: MutableList<MatOfPoint>
    ) {
        val hull = MatOfInt()
        output.clear()

        for (i in inputContours.indices) {
            val contour = inputContours[i]
            val bb = Imgproc.boundingRect(contour)
            if (bb.width < minWidth || bb.width > maxWidth) continue
            if (bb.height < minHeight || bb.height > maxHeight) continue
            val area = Imgproc.contourArea(contour)
            if (area < minArea) continue
            if (Imgproc.arcLength(MatOfPoint2f(*contour.toArray()), true) < minPerimeter) continue
            Imgproc.convexHull(contour, hull)
            val mopHull = MatOfPoint()
            mopHull.create(hull.size().height.toInt(), 1, CvType.CV_32SC2)
            var j = 0
            while (j < hull.size().height) {
                val index = hull.get(j, 0)[0].toInt()
                val point = doubleArrayOf(contour.get(index, 0)[0], contour.get(index, 0)[1])
                mopHull.put(j, 0, *point)
                j++
            }
            val solid = 100 * area / Imgproc.contourArea(mopHull)
            if (solid < solidity[0] || solid > solidity[1]) continue
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount) continue
            val ratio = bb.width / bb.height.toDouble()
            if (ratio < minRatio || ratio > maxRatio) continue
            output.add(contour)
        }
    }

    companion object {
        const val TIME_OUT_DURATION = 2000

        init {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        }
    }
}