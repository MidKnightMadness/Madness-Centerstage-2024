package org.firstinspires.ftc.teamcode.Localization;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class Processor implements VisionProcessor {
    //team prop:
    //red rectangular box
    //probably with dark words on the outside
    int elementLocation;
    int[][] boundsLeft = {};
    int[][] boundsMid =  {};
    int[][] boundsRight = {};

    double minimumThresholdRed;
    //possible for the dark words
    //double minimumThresholdBlack;

    boolean detected;
    double numberOfR;



    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
