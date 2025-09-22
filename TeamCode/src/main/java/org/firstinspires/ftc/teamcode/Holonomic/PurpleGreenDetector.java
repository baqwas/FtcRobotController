/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Holonomic;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Moments; // N.B. .imgproc, not .core
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PurpleGreenDetector extends OpenCvPipeline {

    // Define the color ranges for purple and green in HSV
    private static final Scalar PURPLE_LOWER = new Scalar(125, 50, 50);
    private static final Scalar PURPLE_UPPER = new Scalar(160, 255, 255);
    private static final Scalar GREEN_LOWER = new Scalar(40, 50, 50);
    private static final Scalar GREEN_UPPER = new Scalar(80, 255, 255);

    // List to store detected obstacle centers
    private final List<Point> detectedObstacleCenters = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        // Convert the input image from RGB to HSV color space
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Mats for detected colors
        Mat purpleMask = new Mat();
        Mat greenMask = new Mat();

        // Threshold the HSV image to get only purple and green colors
        Core.inRange(hsv, PURPLE_LOWER, PURPLE_UPPER, purpleMask);
        Core.inRange(hsv, GREEN_LOWER, GREEN_UPPER, greenMask);

        // Combine the masks
        Mat combinedMask = new Mat();
        Core.add(purpleMask, greenMask, combinedMask);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(combinedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Clear previous detections
        detectedObstacleCenters.clear();

        // Iterate through contours to find the centers of the obstacles
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            // Filter out small contours that are likely noise
            if (area > 500) {
                // Get the center of the contour
                Moments moments = Imgproc.moments(contour);
                if (moments.m00 != 0) {
                    int centerX = (int) (moments.m10 / moments.m00);
                    int centerY = (int) (moments.m01 / moments.m00);
                    detectedObstacleCenters.add(new Point(centerX, centerY));
                    // Draw a circle on the original frame for visualization
                    Imgproc.circle(input, new Point(centerX, centerY), 10, new Scalar(0, 255, 0), -1);
                }
            }
        }

        // Cleanup
        hsv.release();
        purpleMask.release();
        greenMask.release();
        combinedMask.release();
        hierarchy.release();

        return input; // Return the frame with circles drawn on it
    }

    public List<Point> getDetectedObstacleCenters() {
        return detectedObstacleCenters;
    }
}