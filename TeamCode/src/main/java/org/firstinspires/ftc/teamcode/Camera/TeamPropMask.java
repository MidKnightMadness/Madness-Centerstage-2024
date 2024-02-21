package org.firstinspires.ftc.teamcode.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.Pose;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.SpikeMarkPositions;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.CameraModes;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


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


    // blue color bounds
    Scalar blueLower = new Scalar(85, 90, 90);
    Scalar blueUpper = new Scalar(145, 255, 255);

    // red color bounds
    Scalar redLower = new Scalar(0, 100, 100);
    Scalar redUpper = new Scalar(15, 255, 255);
    Scalar redLower2 = new Scalar(160, 100, 100);
    Scalar redUpper2 = new Scalar(180, 255, 255);


//    Rect leftRect = RectangleFactory.generateRectFromPercentages(width, height, 0, 50, 27, 100);
//    Rect rightRect = RectangleFactory.generateRectFromPercentages(width, height, 36, 46, 74, 70);
//    Rect centerRect = RectangleFactory.generateRectFromPercentages(width, height, 73, 50, 100, 100);

    Rect leftRect = new Rect(90, 200, 95, 75);
    Rect rightRect = new Rect(510, 200, 95, 75);
    Rect centerRect = new Rect(300, 185, 95, 75);

    Map<SpikeMarkPositions, Pose> spikeMarkScoringOffsets = new HashMap<SpikeMarkPositions, Pose>() {{
        put(SpikeMarkPositions.LEFT, new Pose(10, 10, Math.PI/2));
    }};

//    red {
//        left: vector2
//        center: vector2
//        right: vector2
//    }
//
//    blue {
//        left: vector2
//        center: vector2
//        right: vector2
//    }

    public double[][] coordinates = {
            {1,0,24,42},
            {1,1,36,48},
            {1,2,48,42},

            {2,0,72,42},
            {2,1,84,48},
            {2,2,96,42},

            {3,0,24,102},
            {3,1,36,96},
            {3,2,48,102},

            {4,0,72,102},
            {4,1,84,96},
            {4,2,96,102}
    };

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



    public void setMode(CameraModes mode) {
        this.mode = mode;
    }

    String getMode() {
        return mode == CameraModes.RED ? "red" : mode == CameraModes.BLUE ? "blue" : "unknown";
    }

    Mat getGrayScaleHsv(Mat hsvMat) {
        ArrayList<Mat> channels = new ArrayList<>(3);
        Core.split(hsvMat, channels);
        return channels.get(0);
    }

    int teamPropPosition = 0;
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
            teamPropPosition = 0;
        }
        else if (right > left && right > center) {
            rightColor = detectedRectColor;
            position = SpikeMarkPositions.RIGHT;
            teamPropPosition = 1;
        }
        else {
            centerColor = detectedRectColor;
            position = SpikeMarkPositions.CENTER;
            teamPropPosition = 2;
        }

        telemetry.clear();


        Imgproc.rectangle(output, leftRect, leftColor, 4);
        Imgproc.rectangle(output, rightRect, rightColor, 4);
        Imgproc.rectangle(output, centerRect, centerColor, 4);

        return output;
    }

    public int getTeamPropPosition() {
        return teamPropPosition;
    }

    public SpikeMarkPositions getSpikeMarkPosition() {
        return position;
    }

    public Vector2 getCoordinates(int teamPropPosition, int robotPosition) {
        for(int i = 0;i<coordinates.length;i+=3){
            if(robotPosition==i){

                for(int j =0;j<3;j++){
                    if(coordinates[i+j][1]==teamPropPosition){

                        return new Vector2(coordinates[i+j][3],coordinates[i+j][4]);

                    }
                }
            }

        }

        return null;
    }
}
