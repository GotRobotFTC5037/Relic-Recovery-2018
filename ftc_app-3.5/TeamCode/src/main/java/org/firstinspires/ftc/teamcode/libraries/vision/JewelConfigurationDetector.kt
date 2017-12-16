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

    private val resizeOutput = Mat()
    private val blurOutput = Mat()
    private val hsvOutput = Mat()

    private val whiteThresholdOutput = Mat()
    private val whiteFindContoursOutput = ArrayList<MatOfPoint>()

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

    private val detectedObjectsOutput = Mat()

    private var shouldShowDetectedJewelPositions = true

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
        findBlobs(redDilateOutput, 500.0, doubleArrayOf(0.75, 1.0), false, findRedBlobsOutput)
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
        findBlobs(blueDilateOutput, 500.0, doubleArrayOf(0.75, 1.0), false, findBlueBlobsOutput)
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
        val findContoursOutput = ArrayList<MatOfPoint>()
        val filterContoursOutput = ArrayList<MatOfPoint>()
        Imgproc.findContours(whiteThresholdOutput, findContoursOutput, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
        filterContours(findContoursOutput,
                0.0, 0.0,
                5.0, 40.0,
                10.0, 90.0,
                doubleArrayOf(60.0, 100.0),
                30.0, 0.0,
                0.25, 0.75,
                filterContoursOutput)
        convexHulls(filterContoursOutput, whiteFindContoursOutput)

        return if (whiteFindContoursOutput.isNotEmpty()) {
            val boundingRectangle = Imgproc.boundingRect(whiteFindContoursOutput[0])
            Point(boundingRectangle.x.toDouble() + (boundingRectangle.width / 2), boundingRectangle.y.toDouble() + (boundingRectangle.height / 2))
        } else {
             null
        }

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

        val originalSize = rgba.size()
        Imgproc.resize(rgba, resizeOutput, Size(320.0, 180.0), 0.0, 0.0, Imgproc.INTER_AREA)

        // Blur the image to get rid of noise.
        Imgproc.blur(resizeOutput, blurOutput, Size(7.0, 7.0))
        Imgproc.cvtColor(blurOutput, hsvOutput, Imgproc.COLOR_BGR2HSV)

        // Filter out anything that is not the color of the white line in between the jewels.
        val whiteHueThreshold = doubleArrayOf(0.0, 180.0)
        val whiteSaturationThreshold = doubleArrayOf(0.0, 100.0)
        val whiteValueThreshold = doubleArrayOf(175.0, 255.0)
        Core.inRange(hsvOutput, Scalar(whiteHueThreshold[0], whiteSaturationThreshold[0], whiteValueThreshold[0]), Scalar(whiteHueThreshold[1], whiteSaturationThreshold[1], whiteValueThreshold[1]), whiteThresholdOutput)

        // Filter out anything that is not the color of the blue jewel.
        val bluePositiveHueThreshold = doubleArrayOf(0.0, 40.0)
        val blueNegativeHueThreshold = doubleArrayOf(150.0, 180.0)
        val blueSaturationThreshold = doubleArrayOf(60.0, 255.0)
        val blueValueThreshold = doubleArrayOf(25.0, 255.0)
        Core.inRange(hsvOutput, Scalar(bluePositiveHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), Scalar(bluePositiveHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), bluePositiveThresholdOutput)
        Core.inRange(hsvOutput, Scalar(blueNegativeHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), Scalar(blueNegativeHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), blueNegativeThresholdOutput)
        Core.bitwise_or(bluePositiveThresholdOutput, blueNegativeThresholdOutput, blueThresholdOutput)
        Imgproc.erode(blueThresholdOutput, blueErodeOutput, Mat(), Point(-1.0, -1.0), 1, Core.BORDER_CONSTANT, Scalar(-1.0))
        Imgproc.dilate(blueErodeOutput, blueDilateOutput, Mat(), Point(-1.0, -1.0), 4, Core.BORDER_CONSTANT, Scalar(-1.0))

        // Filter out anything that is not the color of the red jewel.
        val redHueThreshold = doubleArrayOf(110.0, 140.0)
        val redSaturationThreshold = doubleArrayOf(150.0, 255.0)
        val redValueThreshold = doubleArrayOf(25.0, 255.0)
        Core.inRange(hsvOutput, Scalar(redHueThreshold[0], redSaturationThreshold[0], redValueThreshold[0]), Scalar(redHueThreshold[1], redSaturationThreshold[1], redValueThreshold[1]), redThresholdOutput)
        Imgproc.erode(redThresholdOutput, redErodeOutput, Mat(), Point(-1.0, -1.0), 1, Core.BORDER_CONSTANT, Scalar(-1.0))
        Imgproc.dilate(redErodeOutput, redDilateOutput, Mat(), Point(-1.0, -1.0), 4, Core.BORDER_CONSTANT, Scalar(-1.0))

        resizeOutput.copyTo(detectedObjectsOutput)

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
                val yellowScalar = Scalar(255.0, 255.0, 0.0)
                Imgproc.circle(detectedObjectsOutput, Point(x, y), 5, yellowScalar, 5)
            }

            // Clearly identify the red jewel on the screen.
            if (redKeyPoint != null) {
                val x = redKeyPoint.pt.x
                val y = redKeyPoint.pt.y
                val size = redKeyPoint.size
                val radius = size / 2
                val redScalar = Scalar(255.0, 0.0, 0.0)
                Imgproc.rectangle(detectedObjectsOutput, Point(x - radius, y - radius), Point(x + radius, y + radius), redScalar, 3)
                Imgproc.putText(detectedObjectsOutput, "Red", Point(x - radius, y - 5 - radius), 0, 0.5, redScalar, 2)
            }

            // Clearly identify the blue jewel on the screen.
            if (blueKeyPoint != null) {
                val x = blueKeyPoint.pt.x
                val y = blueKeyPoint.pt.y
                val size = blueKeyPoint.size
                val radius = size / 2
                val blueScalar = Scalar(0.0, 0.0, 255.0)
                Imgproc.rectangle(detectedObjectsOutput, Point(x - radius, y - radius), Point(x + radius, y + radius), blueScalar, 3)
                Imgproc.putText(detectedObjectsOutput, "Blue", Point(x - radius, y - 5 - radius), 0, 0.5, blueScalar, 2)
            }

        }

        Imgproc.resize(detectedObjectsOutput, detectedObjectsOutput, originalSize, 0.0, 0.0, Imgproc.INTER_AREA)

        return detectedObjectsOutput
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

    /**
     * Compute the convex hulls of contours.
     * @param inputContours The contours on which to perform the operation.
     * @param outputContours The contours where the output will be stored.
     */
    private fun convexHulls(inputContours: List<MatOfPoint>, outputContours: java.util.ArrayList<MatOfPoint>) {
        val hull = MatOfInt()
        outputContours.clear()
        for (i in inputContours.indices) {
            val contour = inputContours[i]
            val mopHull = MatOfPoint()
            Imgproc.convexHull(contour, hull)
            mopHull.create(hull.size().height.toInt(), 1, CvType.CV_32SC2)
            var j = 0
            while (j < hull.size().height) {
                val index = hull.get(j, 0)[0].toInt()
                val point = doubleArrayOf(contour.get(index, 0)[0], contour.get(index, 0)[1])
                mopHull.put(j, 0, *point)
                j++
            }
            outputContours.add(mopHull)
        }
    }


    /**
     * Filters out contours that do not meet certain criteria.
     * @param inputContours is the input list of contours
     * @param output is the the output list of contours
     * @param minArea is the minimum area of a contour that will be kept
     * @param minPerimeter is the minimum perimeter of a contour that will be kept
     * @param minWidth minimum width of a contour
     * @param maxWidth maximum width
     * @param minHeight minimum height
     * @param maxHeight maximum height
     * @param solidity the minimum and maximum solidity of a contour
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio minimum ratio of width to height
     * @param maxRatio maximum ratio of width to height
     */
    private fun filterContours(inputContours: List<MatOfPoint>,
                               minArea: Double, minPerimeter: Double,
                               minWidth: Double, maxWidth: Double,
                               minHeight: Double, maxHeight: Double,
                               solidity: DoubleArray,
                               maxVertexCount: Double, minVertexCount: Double,
                               minRatio: Double, maxRatio: Double,
                               output: MutableList<MatOfPoint>) {

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
}