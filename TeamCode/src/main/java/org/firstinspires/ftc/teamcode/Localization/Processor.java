package org.firstinspires.ftc.teamcode.Localization;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class Processor implements VisionProcessor{
    int elementLocation = 0; // 0 is left, 1 is middle, 2 is right
    //10000 hd
    int [][] boundsLeft = {{240, 240}, {480, 480}};
    int [][] boundsMiddle = {{520,520}, {720,720}};
    int [][] boundsRight = {{800, 800}, {1100,1100}};
    final double minimimumThresholdRPercentage = 0.3;
    final double minimumThresholdBPercentage = 0.4;

    final int minimumThresholdRNumber = 30;
    final int minimumThresholdBNumber = 40;

     final boolean redIsTrue = true;
     final boolean blueIsTrue = false;
    //arbitary values

    //changeable values below
    public double R = 0;
    public double B  = 0;
    @Override           //1280   by 1280
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }


    public boolean processFrame2(Mat frame, long captureTimeNanos) {


        for(int i = boundsLeft[0][1]; i < boundsLeft[1][1] + 1; i++){  //240 - 481
            for(int j = boundsLeft[0][0]; j < boundsLeft[1][0] + 1; j++){//240 - 481
                if(frame.get(i,j)[0]>=minimimumThresholdRPercentage){
                    R++;
                }
                if(frame.get(i,j)[2]>=minimumThresholdBPercentage){
                    B++;
                }
            }
        }
        //inside the for loop which spans over every pixel
        if(redIsTrue==true){
            if(R>minimumThresholdRNumber) {
                return true;
            }
            return false;
        }
        //otherwise if it is blue
        if(blueIsTrue==true) {
            if (B > minimumThresholdBNumber) {
                return true;
            }
            return false;
        }

        return false;

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
