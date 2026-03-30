package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.config.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipeline extends OpenCvPipeline {

    private Point largestContourCenter = new Point(-1,-1);
    private double largestContourArea = 0.0;

    // The colors(HSV) the pipeline will be detecting (refer to readme about HSV)
    private Scalar lowerHSV;
    private Scalar upperHSV;

    // Constructor is only to set the color ranges when initializing
    public ColorDetectionPipeline(Scalar lowerHSV, Scalar upperHSV){
	   this.lowerHSV = lowerHSV;
	   this.upperHSV = upperHSV;
    }

    // Pipeline begin
    @Override
    public Mat processFrame(Mat input) {
	   // Blur the input image to reduce noise and get a more accurate contour
	   Mat blurred = new Mat();
	   Imgproc.GaussianBlur(input, blurred, new Size(5,5),0);

	   // Convert the input frame into HSV color space
	   Mat hsv = new Mat();
	   Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

	   // Create a binary mask for the specified colo range
	   Mat mask = new Mat();
	   Core.inRange(hsv, lowerHSV, upperHSV, mask);

	   // Apply morphological operations to clean up the mask
	   // Basically Dilates the image to close gaps between contours
	   Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3));
	   Imgproc.dilate(mask,mask,kernel);
	   Imgproc.erode(mask, mask, kernel);

	   // Find contours in the binary mask
	   List<MatOfPoint> contours = new ArrayList<>();
	   Mat hierarchy = new Mat();
	   Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

	   // Variables to track largestContour
	   largestContourArea = 0.0;
	   MatOfPoint largestContour = null;

	   // Find the largestContour
	   for(MatOfPoint contour: contours){
		  double area = Imgproc.contourArea(contour);
		  if(area > largestContourArea){
			 largestContourArea = area;
			 largestContour = contour;
		  }
	   }

	   // If there is a largest contour, then calculate its center
	   if (largestContour != null) {
		  Moments moments = Imgproc.moments(largestContour);
		  // Calculate the center (cx, cy) using image moments
		  int cx = (int) ((int) moments.get_m10() / moments.get_m00());
		  int cy = (int) ((int) moments.get_m01() / moments.get_m00());

		  largestContourCenter = new Point(cx, cy);

		  // DEBUG: Draw the largest contour and its center
		  Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 0), 3);
		  Imgproc.circle(input, largestContourCenter, 5, new Scalar(255, 0, 0), -1);

		  // DEBUG: Draw Coordinates at center coordinates
		  String coordinateText = "X: " + cx + " Y: " + cy;
		  Scalar textColor = new Scalar(255, 255, 255);
		  Point coordsOrigin = new Point(largestContourCenter.x - 20, largestContourCenter.y + 10);
		  Imgproc.putText(input, coordinateText, coordsOrigin, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, textColor, 2);

		  // DEGUG: Draw Target Area
		  Imgproc.rectangle(input, Config.Vision.TARGET_AREA.tl(), Config.Vision.TARGET_AREA.br(), new Scalar(0, 255, 255), 2);

	   } else {
		  // If nothing detected return point to default;
		  largestContourCenter = new Point(-1, -1);
	   }

	   // Free Memory
	   hsv.release();
	   mask.release();
	   hierarchy.release();

	   return input;
    }

    // Get the largest contour center as Point
    public Point getLargestContourCenter() {
	   return largestContourCenter;
    }

    // Get the largest contour area
    public double getLargestContourArea() {
	   return largestContourArea;
    }

    // Set the color range to detect
    public void setColorRange(Scalar lower, Scalar upper){
	   this.lowerHSV = lower;
	   this.upperHSV = upper;
    }

    public boolean isCenterInTargetArea(){
	   return Config.Vision.TARGET_AREA.contains(largestContourCenter);
    }
}
