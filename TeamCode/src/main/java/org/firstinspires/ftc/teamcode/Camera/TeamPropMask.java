package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.SpikeMarkPositions;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.CameraModes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class TeamPropMask extends OpenCvPipeline {
    Mat hsvMat = new Mat();
    Mat output = new Mat();

    CameraModes mode = CameraModes.RED;
    SpikeMarkPositions position = SpikeMarkPositions.LEFT;

    SpikeMarkPositions getPosition() {
        return position;
    }

    int width;
    int height;

    Telemetry telemetry;


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

    public TeamPropMask(int width, int height, Telemetry telemetry) {
        this.width = width;
        this.height = height;
        this.telemetry = telemetry;
    }
    public TeamPropMask(double width, double height, Telemetry telemetry) {
        this.width = (int) width;
        this.height = (int) height;
        this.telemetry = telemetry;
    }

    Scalar defaultRectColor = new Scalar(255, 255, 255); // white
    Scalar detectedRectColor = new Scalar(100, 150, 255); // gray



    void setMode(CameraModes mode) {
        this.mode = mode;
    }

    String getMode() {
        return mode == CameraModes.BLUE ? "red" : mode == CameraModes.RED ? "blue" : "unknown";
    }

    Mat getGrayScaleHsv(Mat hsvMat) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(hsvMat, channels);
        return channels.get(0);
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        if (mode == CameraModes.RED) {
            Mat redOutput1 = new Mat();
            Mat redOutput2 = new Mat();

            // red bounds
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


        Mat leftRectMat = output.submat(leftRect);
        Mat rightRectMat = output.submat(rightRect);
        Mat centerRectMat = output.submat(centerRect);

        Scalar leftAvg = Core.mean(leftRectMat);
        Scalar rightAvg = Core.mean(rightRectMat);
        Scalar centerAvg = Core.mean(centerRectMat);

        double left = leftAvg.val[0];
        double right = rightAvg.val[0];
        double center = centerAvg.val[0];

        if (left > right && left > center) {
            leftColor = detectedRectColor;
            position = SpikeMarkPositions.LEFT;
        }
        else if (right > left && right > center) {
            rightColor = detectedRectColor;
            position = SpikeMarkPositions.RIGHT;
        }
        else {
            centerColor = detectedRectColor;
            position = SpikeMarkPositions.CENTER;
        }

        telemetry.clear();

        telemetry.addData("Left AVG", leftAvg);
        telemetry.addData("right AVG", rightAvg);
        telemetry.addData("center AVG", centerAvg);

        Imgproc.rectangle(output, leftRect, leftColor, 4);
        Imgproc.rectangle(output, rightRect, rightColor, 4);
        Imgproc.rectangle(output, centerRect, centerColor, 4);

        return output;
    }
}
