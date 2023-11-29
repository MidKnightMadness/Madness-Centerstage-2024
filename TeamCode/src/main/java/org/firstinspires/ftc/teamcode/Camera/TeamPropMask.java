package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

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


    Rect leftRect = RectangleFactory.generateRectFromPercentages(width, height, 0, 50, 27, 100);
    Rect rightRect = RectangleFactory.generateRectFromPercentages(width, height, 36, 46, 74, 70);
    Rect centerRect = RectangleFactory.generateRectFromPercentages(width, height, 73, 50, 100, 100);

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

    Scalar rectColor = new Scalar(255, 255, 255);

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


        Mat leftRectMat = hsvMat.submat(leftRect);
        Mat rightRectMat = hsvMat.submat(rightRect);
        Mat centerRectMat = hsvMat.submat(centerRect);

        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);
        Imgproc.rectangle(output, centerRect, rectColor, 2);

        return output;
    }
}
