package org.firstinspires.ftc.teamcode.Camera;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class TeamPropMask extends OpenCvPipeline {
    Mat hsvMat = new Mat();
    Mat output = new Mat();

    int width;
    int height;


    // blue bounds
    Scalar blueLower = new Scalar(100, 100, 100);
    Scalar blueUpper = new Scalar(140, 255, 255);

    // red bounds
    Scalar redLower = new Scalar(0, 100, 100);
    Scalar redUpper = new Scalar(15, 255, 255);
    Scalar redLower2 = new Scalar(160, 100, 100);
    Scalar redUpper2 = new Scalar(180, 255, 255);


//    Rect leftRect = RectangleFactory.generateRectFromPercentages(width, height, 0, 50, 27, 100);
//    Rect rightRect = RectangleFactory.generateRectFromPercentages(width, height, 36, 46, 74, 70);
//    Rect centerRect = RectangleFactory.generateRectFromPercentages(width, height, 73, 50, 100, 100);

    Rect leftRect = new Rect(90, 150, 95, 70);
    Rect rightRect = new Rect(300, 135, 70, 55);
    Rect centerRect = new Rect(495, 130, 95, 75);

    // 0: red
    // 1: blue
    int mode = 0;

    public TeamPropMask(int width, int height) {
        this.width = width;
        this.height = height;
    }
    public TeamPropMask(double width, double height) {
        this.width = (int) width;
        this.height = (int) height;
    }

    Scalar defaultRectColor = new Scalar(10, 255, 255);
    Scalar detectedRectColor = new Scalar(50, 255, 255);

    void setMode(String mode) {
        if (mode.equals("blue")) {
            this.mode = 1;
        }
        else if (mode.equals("red")) {
            this.mode = 0;
        }
    }

    void setMode(int mode) {
        if (mode == 1 || mode == 0) {
            this.mode = mode;
        }
    }

    String getMode() {
        return mode == 0 ? "red" : mode == 1 ? "blue" : "unknown";
    }

    Mat getGrayScaleHsv(Mat hsvMat) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(hsvMat, channels);
        return channels.get(2);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (mode == 0) {
            Mat redOutput1 = new Mat();
            Mat redOutput2 = new Mat();

            // red
            Core.inRange(hsvMat, redLower, redUpper, redOutput1);
            Core.inRange(hsvMat, redLower2, redUpper2, redOutput2);
            Core.bitwise_or(redOutput1, redOutput2, output);
        }
        else {
            Core.inRange(hsvMat, blueLower, blueUpper, output);
        }

        Scalar leftColor = defaultRectColor;
        Scalar rightColor  = defaultRectColor;
        Scalar centerColor = defaultRectColor;

        // convert output to grayscale (V value)
//        Imgproc.cvtColor(output, output, Imgproc.COLOR_HSV2RGB);
//        Imgproc.cvtColor(output, output, Imgproc.COLOR_RGB2GRAY);

        Mat leftRectMat = getGrayScaleHsv(output.submat(leftRect));
        Mat rightRectMat = getGrayScaleHsv(output.submat(leftRect));
        Mat centerRectMat = getGrayScaleHsv(output.submat(leftRect));

        int left = Core.countNonZero(leftRectMat);
        int right = Core.countNonZero(rightRectMat);
        int center = Core.countNonZero(centerRectMat);

//        int max = Arrays.stream(new int[] {right, left, center}).max().getAsInt();
        if (left > right && left > center) {
            leftColor = detectedRectColor;
        }
        else if (right > left && right > center) {
            rightColor = detectedRectColor;
        }
        else {
            centerColor = detectedRectColor;
        }

        Imgproc.rectangle(output, leftRect, leftColor, 4);
        Imgproc.rectangle(output, rightRect, rightColor, 4);
        Imgproc.rectangle(output, centerRect, centerColor, 4);

        return output;
    }
}
