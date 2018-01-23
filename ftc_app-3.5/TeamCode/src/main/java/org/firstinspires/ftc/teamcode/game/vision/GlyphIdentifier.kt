package org.firstinspires.ftc.teamcode.game.vision

import com.vuforia.CameraDevice
import org.corningrobotics.enderbots.endercv.OpenCVPipeline
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import java.util.*
import kotlin.math.acos
import kotlin.math.min
import kotlin.math.roundToInt

class GlyphIdentifier : OpenCVPipeline() {

    companion object {
        const val GLYPH_SIZE = 15.2

        init {
            System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        }
    }

    private val resizeImageOutput = Mat()
    private val blurOutput = Mat()

    private val edgeDetectionOutput = Mat()
    private val edgeDilationOutput = Mat()

    private val brownGlyphColorThresholdOutput = Mat()
    private val greyGlyphColorThresholdOutput = Mat()
    private val combinedGlyphColorThresholds = Mat()
    private val glyphRemovedEdgesOutput = Mat()

    private val findContoursOutput = ArrayList<MatOfPoint>()
    private val convexHullsOutput = ArrayList<MatOfPoint>()
    private val filterContoursOutput = ArrayList<MatOfPoint>()

    private val identifiedGlyphsOutput = Mat()

    private val cameraFocalLength: Double
        get() = CameraDevice.getInstance().cameraCalibration.focalLength.data[0].toDouble() / 10

    override fun processFrame(rgba: Mat?, gray: Mat?): Mat {

        if (rgba != null) {
            // Resize the image and remove noise.
            val originalSize = rgba.size()
            Imgproc.resize(rgba, resizeImageOutput, Size(320.0, 180.0), 0.0, 0.0, Imgproc.INTER_AREA)
            Imgproc.cvtColor(resizeImageOutput, resizeImageOutput, Imgproc.COLOR_BGRA2BGR)
            Imgproc.bilateralFilter(resizeImageOutput.clone(), blurOutput, -1, 4.0, 4.0)

            // Find the edges in the image.
            Imgproc.Canny(blurOutput, edgeDetectionOutput, 15.0, 45.0, 3, true)
            Imgproc.dilate(edgeDetectionOutput, edgeDilationOutput, Mat(), Point(-1.0, -1.0), 2, Core.BORDER_CONSTANT, Scalar(-1.0))

            // Filter out things that probably are not glyphs.
            val brownHue = doubleArrayOf(90.0, 150.0)
            val brownSat = doubleArrayOf(20.0, 115.0)
            val brownLum = doubleArrayOf(0.0, 125.0)
            Imgproc.cvtColor(blurOutput, brownGlyphColorThresholdOutput, Imgproc.COLOR_BGR2HLS)
            Core.inRange(brownGlyphColorThresholdOutput, Scalar(brownHue[0], brownLum[0], brownSat[0]), Scalar(brownHue[1], brownLum[1], brownSat[1]), brownGlyphColorThresholdOutput)

            val greyHue = doubleArrayOf(0.0, 180.0)
            val greySat = doubleArrayOf(0.0, 40.0)
            val greyVal = doubleArrayOf(40.0, 255.0)
            Imgproc.cvtColor(blurOutput, greyGlyphColorThresholdOutput, Imgproc.COLOR_BGR2HSV)
            Core.inRange(greyGlyphColorThresholdOutput, Scalar(greyHue[0], greySat[0], greyVal[0]), Scalar(greyHue[1], greySat[1], greyVal[1]), greyGlyphColorThresholdOutput)

            // Add the brown and grey glyph filters together and remove the edges.
            // TODO: Identify the different glyph colors individually.
            Core.bitwise_xor(brownGlyphColorThresholdOutput, greyGlyphColorThresholdOutput, combinedGlyphColorThresholds)
            Core.subtract(combinedGlyphColorThresholds, edgeDilationOutput, glyphRemovedEdgesOutput)

            // Identify the glyph faces.
            findContoursOutput.clear()
            Imgproc.findContours(glyphRemovedEdgesOutput, findContoursOutput, Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE)
            convexHulls(findContoursOutput, convexHullsOutput)
            filterContours(convexHullsOutput,
                    1000.0, 100.0,
                    20.0, 160.0,
                    20.0, 90.0,
                    doubleArrayOf(0.0, 100.0),
                    30.0, 4.0,
                    0.25, 1.75,
                    filterContoursOutput)


            // Show the identified glyphs on the screen.
            resizeImageOutput.copyTo(identifiedGlyphsOutput)

            val yellowScalar = Scalar(255.0, 255.0, 0.0)
            for (contour in filterContoursOutput) {
                val contourBoundingRectangle = Imgproc.boundingRect(contour)
                val xPosition = contourBoundingRectangle.x.toDouble()
                val yPosition = contourBoundingRectangle.y.toDouble()
                val width = contourBoundingRectangle.width
                val height = contourBoundingRectangle.height
                val angle: Int = (acos(min((width / height).toDouble(), 1.0)) * Math.PI / 180).roundToInt()
                Imgproc.rectangle(identifiedGlyphsOutput, Point(xPosition, yPosition), Point(xPosition + width, yPosition + height), yellowScalar, 3)
                Imgproc.putText(identifiedGlyphsOutput, "$angle°", Point(xPosition, yPosition - 4), 0, 0.5, yellowScalar, 2)
            }

            Imgproc.resize(identifiedGlyphsOutput, identifiedGlyphsOutput, originalSize, 0.0, 0.0, Imgproc.INTER_AREA)

            return identifiedGlyphsOutput

        } else {
            return Mat()
        }

    }


    /**
     * Compute the convex hulls of contours.
     * @param inputContours The contours on which to perform the operation.
     * @param outputContours The contours where the sumOutput will be stored.
     */
    private fun convexHulls(inputContours: List<MatOfPoint>, outputContours: ArrayList<MatOfPoint>) {
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
     * @param output is the the sumOutput list of contours
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

