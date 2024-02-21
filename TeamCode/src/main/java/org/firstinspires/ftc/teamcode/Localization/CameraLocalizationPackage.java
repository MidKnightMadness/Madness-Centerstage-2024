package org.firstinspires.ftc.teamcode.Localization;

import android.graphics.Canvas;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/*
Upon init, includes following forms of localization that can be turned on and off:
1. Team Prop Detection
2. April Tag Detection
3. White Pixel Detection

Each uses a modified tfod pipeline
Note: pixel detection can be modified from default given to yield much faster runtimes
 */
public class CameraLocalizationPackage {
    // Using different devices
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private boolean USE_TEAM_PROP_DETECTION = true;
    private boolean USE_APRIL_TAG_DETECTION = true;
    private boolean USE_WHITE_PICKLE_DETECTION = true;

    // Processor and visionportal objects,  visionportal manages running of each pipeline from inserted frames
    // Have yet to combine everything into one pipeline, needs a bit more code surgery and piracy
    private TfodProcessor teamPropProcessor;
    private AprilTagProcessor aprilTag;
    private TfodProcessor pickleRick; // pickle detector
    private VisionPortal myVisionPortal;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    // Variables and objects for team prop detection
    Mat hsvMat = new Mat();
    Mat output = new Mat();

    CameraEnums.CameraModes mode = CameraEnums.CameraModes.RED;
    CameraEnums.SpikeMarkPositions position = CameraEnums.SpikeMarkPositions.LEFT;

    // blue color bounds
    Scalar blueLower = new Scalar(85, 90, 90);
    Scalar blueUpper = new Scalar(145, 255, 255);

    // red color bounds
    Scalar redLower = new Scalar(0, 100, 100);
    Scalar redUpper = new Scalar(15, 255, 255);
    Scalar redLower2 = new Scalar(160, 100, 100);
    Scalar redUpper2 = new Scalar(180, 255, 255);

    Rect leftRect = new Rect(90, 200, 95, 75);
    Rect rightRect = new Rect(510, 200, 95, 75);
    Rect centerRect = new Rect(300, 185, 95, 75);

    Scalar defaultRectColor = new Scalar(255, 255, 255); // white
    Scalar detectedRectColor = new Scalar(100, 150, 255); // gray

    int [] teamPropPosition = {0, 0, 0}; // Counts up nost probable position to avoid it changing last minute

    // Variables for April Tag Localization
    public final double[][] APRIL_TAG_COORDS = { // hardcoded
            {135d, 115d},//id 1
            {135d, 109d},//id 2
            {135d, 103d},//id 3
            {135d, 41d},//id 4
            {135d, 35d},//id 5
            {135d, 29d},//id 6
            {0d, 114d},//id 7 not necesarily accurate yet
            {0d, 108d},//id 8 not necesarily accurate yet
            {0d, 36d},//id 9 not necesarily accurate yet
            {0d, 30d}//id 10 not necesarily accurate y
    };
    double [] CAMERA_RELATIVE_COORDINATES = {0d, 0d};
    double heading = 0d;
    double [] cameraCoordinates = {0d, 0d};
    double [] robotCoordinates = {0d, 0d}; // Haven't yet needed this functionality, just tuning position of camera for now
    double rangeCoefficient = 0.0;

    // Ignore these for now
    private double [][] TAG_RANGE_CORRECTIONS = { // A, B, C
            // Perceived range = A * exp(C * range) + B
            {2.352 * 10000d, -2.351 * 10000d, 4.179 / 100000d}, // ID 1
            {6503d, -6502d, 0.0001502}, // ID 2
            {3909d, -3908d, 0.0002501}, // ID 3
            {298.8, -297.2, 0.003194}, // ID 4
            {-1.282 * 10000d, 1.282 * 10000d, -7.845 / 100000d}, // ID 5
            {-837.2, 837.6, -0.001248}, // ID 6
    };

    // Builds all detectors on init, by default, only uses team prop detector to be faster
    public CameraLocalizationPackage(HardwareMap hardwareMap, Telemetry telemetry, String webcamName){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        initVisionPortal(webcamName);

        USE_TEAM_PROP_DETECTION = true;
        USE_APRIL_TAG_DETECTION = false;
        USE_WHITE_PICKLE_DETECTION = false;

        myVisionPortal.setProcessorEnabled(teamPropProcessor, true);
        myVisionPortal.setProcessorEnabled(aprilTag, false);
        myVisionPortal.setProcessorEnabled(pickleRick, false);
    }
    private void initVisionPortal(String webcamName){
        // Configure team prop detector
        teamPropProcessor = new TfodProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {

            }

            @Override
            public Object processFrame(Mat input, long captureTimeNanos) {
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

                if (mode == CameraEnums.CameraModes.RED) {
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
                    position = CameraEnums.SpikeMarkPositions.LEFT;
                    teamPropPosition [0]++;
                }
                else if (right > left && right > center) {
                    rightColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.RIGHT;
                    teamPropPosition [1]++;
                }
                else {
                    centerColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.CENTER;
                    teamPropPosition [2]++;
                }


                Imgproc.rectangle(output, leftRect, leftColor, 4);
                Imgproc.rectangle(output, rightRect, rightColor, 4);
                Imgproc.rectangle(output, centerRect, centerColor, 4);

                return output;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

            }

            @Override
            public void setMinResultConfidence(float minResultConfidence) {

            }

            @Override
            public void setClippingMargins(int left, int top, int right, int bottom) {

            }

            @Override
            public void setZoom(double magnification) {

            }

            @Override
            public List<Recognition> getRecognitions() {
                return null;
            }

            @Override
            public List<Recognition> getFreshRecognitions() {
                return null;
            }

            @Override
            public void shutdown() {

            }
        };

        // Configure april tag detector
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Configure pickle detector
        pickleRick = new TfodProcessor.Builder().build(); // Defaults to this

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .addProcessors(teamPropProcessor, aprilTag)//, pickleRick)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(teamPropProcessor, aprilTag)//, pickleRick)
                    .build();
        }
    }
    public CameraEnums.SpikeMarkPositions getSpikeMarkPosition(){
        if(teamPropPosition [0] > teamPropPosition [1] && teamPropPosition [0] > teamPropPosition [2]){ // Left
            return CameraEnums.SpikeMarkPositions.LEFT;
        }else if(teamPropPosition [1] > teamPropPosition [0] && teamPropPosition [1] > teamPropPosition [2]){ // Right
            return CameraEnums.SpikeMarkPositions.RIGHT;
        }else{
            return CameraEnums.SpikeMarkPositions.CENTER;
        }
    }

    public double [] getCameraCoords(double robotHeading, double currentX, double currentY){ // Camera coordinates, relative to field; x, y, heading
        // Reset calculation variables
        rangeCoefficient = 0d;
        heading = 0d;
        cameraCoordinates [0] = 0d;
        cameraCoordinates [1] = 0d;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(currentDetections.size() == 0 || currentDetections == null){ // Works for all tags!
            return new double [] {currentX, currentY, robotHeading};
        }

        for(AprilTagDetection detection : currentDetections) { // Calculate range coefficient
//            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);

            if(detection.id == 3 || detection.id == 4) {
                rangeCoefficient += 1d /
                        (detection.ftcPose.range * detection.ftcPose.range);
            }else {
                rangeCoefficient += 0.1 /
                        (detection.ftcPose.range * detection.ftcPose.range);
            }
        }

        for(AprilTagDetection detection : currentDetections) {
//            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);

            if(detection.id == 3 || detection.id == 4) {
                heading += (detection.ftcPose.yaw * 1d) /
                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else if(detection.id > 6) {
                heading += ((detection.ftcPose.yaw + Math.PI) * 0.1) /
                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else {
                heading += (detection.ftcPose.yaw * 0.1) /
                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }
        }

        for(AprilTagDetection detection : currentDetections) {
//            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);

            if(detection.id == 3 || detection.id == 4) {
                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 1d / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));

                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][1]) * 1d / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else {
                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));

                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][1]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }
        }

        return cameraCoordinates;
    }

    private double correctY(double percievedY){
        return (percievedY + 1.200) / 1.068;
    }

    private double correctX(double percievedX, double actualY){
        return (percievedX - (actualY * 0.06898) + 0.2844) / (-1.0425);
    }

    // Ignore this for now
    public double correctAprilTagError(int id, double perceivedRange){
        return (2.303 * Math.log10((perceivedRange - TAG_RANGE_CORRECTIONS[id - 1][1]) / TAG_RANGE_CORRECTIONS[id - 1][0]) / TAG_RANGE_CORRECTIONS[id - 1][2]);
    }
    public List<Recognition> getDetectedPickles(){ // Still working on this rip
        return pickleRick.getRecognitions();
    }

    // UTILITY FUNCTIONS
    public void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
    }
    public void telemetryTeamPropDetector() {
        telemetry.addData("Detected spike mark position", teamPropPosition);
    }
    public void telemetryPickleDetector() {
        List<Recognition> currentRecognitions = pickleRick.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
    public boolean toggleTeamPropActivation(){
        USE_TEAM_PROP_DETECTION = !USE_TEAM_PROP_DETECTION;
        myVisionPortal.setProcessorEnabled(teamPropProcessor, USE_TEAM_PROP_DETECTION);
        return USE_TEAM_PROP_DETECTION;
    }
    public boolean toggleAprilTagActivation(){
        USE_APRIL_TAG_DETECTION = !USE_APRIL_TAG_DETECTION;
        myVisionPortal.setProcessorEnabled(aprilTag, USE_APRIL_TAG_DETECTION);
        return USE_APRIL_TAG_DETECTION;
    }
    public boolean togglePickleDetectorActivation(){
        USE_WHITE_PICKLE_DETECTION = !USE_WHITE_PICKLE_DETECTION;
        myVisionPortal.setProcessorEnabled(pickleRick, USE_WHITE_PICKLE_DETECTION);
        return USE_WHITE_PICKLE_DETECTION;
    }
    public boolean getTeamPropActivation(){
        return USE_TEAM_PROP_DETECTION;
    }
    public boolean getAprilTagActivation(){
        return USE_APRIL_TAG_DETECTION;
    }
    public boolean getPickleDetectorActivation(){
        return USE_WHITE_PICKLE_DETECTION;
    }
}
