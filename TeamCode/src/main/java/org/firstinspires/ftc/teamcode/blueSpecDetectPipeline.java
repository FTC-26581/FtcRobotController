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

/*
// This class is a custom OpenCvPipeline that detects a sample based on color.
// This detection pipeline was created by FTC Team 26581, Tundra Tech.
// It is originally configured for FTC Into The Deep Season and may need to be modified for your specific use case.
 */
public class blueSpecDetectPipeline extends OpenCvPipeline {
    // These variables will hold the sample’s center and any other data you need
    //Point is a class that holds 2 doubles (x, y) for a 2D point.
    private Point sampleCenter = null;
     
    // Define color thresholds or other parameters for detection.
    // For example, the sample has a yellow color, so we use HSV thresholds:

    // Scalar is a class that holds 3 doubles (H, S, V) for the HSV color space.
    //The following two lines are for setting the HSV values for detecting game pieces
    //Change these as needed.
    private final Scalar lowerHSV = new Scalar(82, 52, 89); // adjust as needed
    private final Scalar upperHSV = new Scalar(179, 255, 255); // adjust as needed

    //processFrame is a method that is called by the OpenCvCamera every frame.
    //The input parameter is the frame from the camera, and the method should return a modified frame.
    //Mat(short for matrix) is a class that holds an image as a matrix of pixels.
    @Override
    public Mat processFrame(Mat input) {
        // Convert the input image from RGB to HSV color space.
        Mat hsv = new Mat();
        // Imgproc is a class that provides image processing functions.
        //Here, we convert the input image to HSV color space.
        //imgproc.cvtColor converts an image from one color space to another.
        //Imgproc.COLOR_RGB2HSV is a constant that specifies the conversion from RGB to HSV.
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
         
        // Threshold the image to isolate your sample’s color
        Mat mask = new Mat();
        //inRange is a function that thresholds an image based on a range of values.
        Core.inRange(hsv, lowerHSV, upperHSV, mask);
         
        // Find contours in the mask.
        //List is an interface that holds a list of objects.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
 
        // Optional: Draw contours for debugging.
        Imgproc.drawContours(input, contours, -1, new Scalar(0, 255, 0), 2);
 
        // Assume the largest contour is the sample (tweak as needed)
        // Rect is a class that holds 4 integers (x, y, width, height) for a rectangle.
        //maxArea is a double that holds the area of the largest contour.
        double maxArea = 0;
        Rect sampleRect = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                sampleRect = Imgproc.boundingRect(contour);//boundingRect is a function that returns a rectangle that bounds a contour.
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
     
    // A getter method so your OpMode can access the detected center.
    public Point getSampleCenter() {
        return sampleCenter;
    }
}
