package org.firstinspires.ftc.teamcode.libraries.vision;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class JewelPipeline extends OpenCVPipeline {

	//Outputs
	private Mat blurOutput = new Mat();
	private Mat hsvOutput = new Mat();
	private Mat hsvThreshold0Output = new Mat();
	private Mat cvErode0Output = new Mat();
	private Mat cvDilate0Output = new Mat();
	private MatOfKeyPoint findBlobs0Output = new MatOfKeyPoint();
	private Mat hsvThreshold1Output = new Mat();
	private Mat cvErode1Output = new Mat();
	private Mat cvDilate1Output = new Mat();
	private MatOfKeyPoint findBlobs1Output = new MatOfKeyPoint();

	private double lastKnownRedPosition = -1;
	private double lastKnownBluePosition = -1;

	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	@Override
	public Mat processFrame(Mat rgba, Mat gray) {
		double kernelSize = 2 * 10.0 + 1;
		Imgproc.blur(rgba, blurOutput, new Size(kernelSize, kernelSize));
		Imgproc.cvtColor(blurOutput, hsvOutput, Imgproc.COLOR_BGR2HSV);

		double[] blueHueThreshold = {0.0, 40.0};
		double[] blueSaturationThreshold = {150.0, 255.0};
		double[] blueValueThreshold = {0.0, 255.0};
		Core.inRange(hsvOutput, new Scalar(blueHueThreshold[0], blueSaturationThreshold[0], blueValueThreshold[0]), new Scalar(blueHueThreshold[1], blueSaturationThreshold[1], blueValueThreshold[1]), hsvThreshold0Output);
		Imgproc.erode(hsvThreshold0Output, cvErode0Output, new Mat(), new Point(-1, -1), 3,  Core.BORDER_CONSTANT, new Scalar(-1));
		Imgproc.dilate(cvErode0Output, cvDilate0Output, new Mat(), new Point(-1, -1), 6, Core.BORDER_CONSTANT, new Scalar(-1));

		double[] redHueThreshold1 = {60.0, 140.0};
		double[] redSaturationThreshold1 = {200.0, 255.0};
		double[] redValueThreshold1 = {0.0, 255.0};
		Core.inRange(hsvOutput, new Scalar(redHueThreshold1[0], redSaturationThreshold1[0], redValueThreshold1[0]), new Scalar(redHueThreshold1[1], redSaturationThreshold1[1], redValueThreshold1[1]), hsvThreshold1Output);
		Imgproc.erode(hsvThreshold1Output, cvErode1Output, new Mat(), new Point(-1, -1), 3,  Core.BORDER_CONSTANT, new Scalar(-1));
		Imgproc.dilate(cvErode1Output, cvDilate1Output, new Mat(), new Point(-1, -1), 6, Core.BORDER_CONSTANT, new Scalar(-1));

		return hsvThreshold1Output;
	}

	public enum JewelPositions {
		RED_BLUE,
		BLUE_RED,
		UNKNOWN
	}

	public double getRedPosition() {
		double[] findBlobs0Circularity = {0.7, 1.0};
		findBlobs(cvDilate0Output, 1000.0, findBlobs0Circularity, false, findBlobs0Output);

		KeyPoint[] redBlobs = findBlobs0Output.toArray();
		if(redBlobs.length > 0) {
			lastKnownRedPosition = redBlobs[0].pt.x;
			return lastKnownRedPosition;
		} else {
			return -1;
		}
	}

	public double getBluePosition() {
		double[] findBlobs1Circularity = {0.7, 1.0};
		findBlobs(cvDilate1Output, 1000.0, findBlobs1Circularity, false, findBlobs1Output);

		KeyPoint[] blueBlobs = findBlobs1Output.toArray();
		if(blueBlobs.length > 0) {
			lastKnownBluePosition = blueBlobs[0].pt.x;
			return lastKnownBluePosition;
		} else {
			return -1;
		}
	}

	public JewelPositions getJewelPositions() {
		final double bluePosition = getBluePosition();
		final double redPosition = getRedPosition();

		if(bluePosition != -1 && redPosition != -1) {
			if(bluePosition > redPosition) {
				return JewelPositions.BLUE_RED;
			} else {
				return JewelPositions.RED_BLUE;
			}
		} else {
			return JewelPositions.UNKNOWN;
		}
	}

	/**
	 * Detects groups of pixels in an image.
	 * @param input The image on which to perform the find blobs.
	 * @param minArea The minimum size of a blob that will be found
	 * @param circularity The minimum and maximum circularity of blobs that will be found
	 * @param darkBlobs The boolean that determines if light or dark blobs are found.
	 * @param blobList The output where the MatOfKeyPoint is stored.
	 */
	private void findBlobs(Mat input, double minArea, double[] circularity,
		Boolean darkBlobs, MatOfKeyPoint blobList) {
		FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
		try {
			File tempFile = File.createTempFile("config", ".xml");

			String config = "<?xml version=\"1.0\"?>\n" +
					"<opencv_storage>\n" +
					"<thresholdStep>10.</thresholdStep>\n" +
					"<minThreshold>50.</minThreshold>\n" +
					"<maxThreshold>220.</maxThreshold>\n" +
					"<minRepeatability>2</minRepeatability>\n" +
					"<minDistBetweenBlobs>10.</minDistBetweenBlobs>\n" +
					"<filterByColor>1</filterByColor>\n" +
					"<blobColor>" +
					(darkBlobs ? 0 : 255) +
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
					"</opencv_storage>\n";

			FileWriter writer;
			writer = new FileWriter(tempFile, false);
			writer.write(config);
			writer.close();
			blobDet.read(tempFile.getPath());
		} catch (IOException e) {
			e.printStackTrace();
		}

		blobDet.detect(input, blobList);
	}
}