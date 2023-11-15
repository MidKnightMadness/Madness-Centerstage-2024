package org.firstinspires.ftc.teamcode.Localization;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class SimpleProcessor implements VisionProcessor{
    int elementLocation = 0; // 0 is left, 1 is middle, 2 is right
    //10000 hd
    int [][] boundsLeft = {{240, 240}, {480, 480}};
    int [][] boundsMiddle = {{520,520}, {720,720}};
    int [][] boundsRight = {{800, 800}, {1100,1100}};
    final double minimimumThresholdRPercentage = 0.3;
   // final double minimumThresholdBPercentage = 0.4;
    final int minimumThresholdRNumber = 90;
    //final int minimumThresholdBNumber = 40;

    //final boolean colorRed = true;
     // boolean blueIsTrue = false;
    //arbitary values

    //changeable values below
    public double rLeft = 0;
    public double rMiddle = 0;
    public double rRight = 0;

    public final double [][] linesCoordinates = {
            //red side front
            //first number = position#
            //second number = team prop placement
            //third number and 4th: x and y
            {1, 1, 24, 30},
            {1, 2, 36, 24},
            {1, 3, 48, 30},

            //red side back
            {2, 1, 72, 30},
            {2, 2, 84, 24},
            {2, 3, 96, 30},

            //blue side front
            {3, 1, 24,102},
            {3, 2, 36, 96},
            {3, 3, 48, 102},

            //blue side back
            {4, 1, 72, 102},
            {4, 2, 84, 96},
            {4, 3, 96, 96}
    };


    //difference in between each of the boxes to find team prop
    public int minimalDifference = 70;

    public int maximumLowerBoundLine = 10;
    //public double B  = 0;
    @Override           //1280   by 1280
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }


    public int processFrame2(Mat frame, long captureTimeNanos) {

        //for the boxes specified in the left, middle, and right boxes, each time
        //goes through each frame and finds the number of pixels with a R value greater than a threshold value

        //if one box has much higher amount than other, spike mark is there

        //left box
        for(int i = boundsLeft[0][1]; i < boundsLeft[1][1] + 1; i++){  //240 - 481
            for(int j = boundsLeft[0][0]; j < boundsLeft[1][0] + 1; j++){//240 - 481
                if(frame.get(i,j)[0]>=minimimumThresholdRPercentage){
                    rLeft++;
                }

            }
        }

        //middle box
        for(int i = boundsMiddle[0][1]; i < boundsMiddle[1][1] + 1; i++){
            for(int j = boundsMiddle[0][0]; j < boundsMiddle[1][0] + 1; j++){
                if(frame.get(i,j)[0]>=minimimumThresholdRPercentage){
                    rMiddle++;
                }

            }
        }

        //right box
        for(int i = boundsRight[0][1]; i < boundsRight[1][1] + 1; i++){
            for(int j = boundsRight[0][0]; j < boundsRight[1][0] + 1; j++){
                if(frame.get(i,j)[0]>=minimimumThresholdRPercentage){
                    rRight++;
                }

            }
        }

        //as long as one side has much more than other sides and other sides are almost the same
        //also that section has to have more pixels with r values than minimum threshold

        if(rLeft >= minimimumThresholdRPercentage &&  rLeft >= rMiddle + minimalDifference && Math.abs(rMiddle-rRight)<=maximumLowerBoundLine){
            return 0;//0 means left
        }

        else if(rMiddle>= minimimumThresholdRPercentage && rMiddle >= rLeft + minimalDifference && Math.abs(rLeft-rRight)<=maximumLowerBoundLine){
            return 1;//1 means middle
        }

        else if(rRight>= minimimumThresholdRPercentage && rRight>=rLeft + minimalDifference && Math.abs(rLeft-rMiddle)<=maximumLowerBoundLine){
            return 2;//2 means right
        }

        else{
            return 3;//4 means not detected
        }



    }

                    //position number means 1 for red Front, 2 for red Back, 3 for blue Front, 4 for blue Back
    public Vector2  getVector(int teamPropPlacement, int positionNumber){
        if(teamPropPlacement!=4) {
            //checking only first two numbers for identification



            for (int i = 0; i < linesCoordinates.length; i += 3) {//checks for the first identification check(positionNumber)
                if (linesCoordinates[i][0] == positionNumber) {



                    for (int z = 0;z<3; z++) {//checks for 2nd identification marker
                        if(linesCoordinates[i+z][1]==teamPropPlacement){

                            //return 3rd and 4th value
                            return new Vector2(linesCoordinates[i][2],linesCoordinates[i][3]);
                        }
                    }


                }
            }
        }

        //if team prop placement is null(meaning does not see team prop)
        return new Vector2(0,0);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
