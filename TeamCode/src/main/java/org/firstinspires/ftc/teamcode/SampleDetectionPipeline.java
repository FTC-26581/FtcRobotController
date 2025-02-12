package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.List;

public class SampleDetectionPipeline extends OpenCvPipeline {
    // These variables will hold the sample’s center and any other data you need
    private Point sampleCenter = null;
     
    // Define your color thresholds or other parameters for detection.
    // For example, if your sample has a unique color, you might use HSV thresholds:
    private Scalar lowerHSV = new Scalar(18, 69, 127); // adjust as needed
    private Scalar upperHSV = new Scalar(179, 255, 255); // adjust as needed

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input image from RGB to HSV color space.
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
         
        // Threshold the image to isolate your sample’s color
        Mat mask = new Mat();
        Core.inRange(hsv, lowerHSV, upperHSV, mask);
         
        // Find contours in the mask.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
 
        // Optional: Draw contours for debugging.
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);
 
        // Assume the largest contour is the sample (tweak as needed)
        double maxArea = 0;
        Rect sampleRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                sampleRect = Imgproc.boundingRect(contour);
            }
        }
 
        if (sampleRect != null) {
            // Calculate the center of the detected sample.
            sampleCenter = new Point(sampleRect.x + sampleRect.width / 2.0,
                                    sampleRect.y + sampleRect.height / 2.0);
            // Draw a circle at the center for visualization.
            Imgproc.circle(input, sampleCenter, 5, new Scalar(255, 0, 0), -1);
        } else {
            sampleCenter = null;
        }
 
        // Return the annotated frame for display on the driver station (if desired).
        return input;
    }
     
    // A getter method so your opmode can access the detected center.
    public Point getSampleCenter() {
        return sampleCenter;
    }
}
