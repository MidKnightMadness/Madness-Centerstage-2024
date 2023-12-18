package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalizer extends Localizer {
    // AUXILLARY VARIABLES =========================================================================
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public double[] sensorCoords1 = {0.0, 0.0}; // For each tag detected, do both localization methods
    public double[] sensorCoords2 = {0.0, 0.0};
    double [] calculationsVector = {0.0, 0.0};

    // VARIABLES COPIED FROM CONCEPT APRIL TAG CLASS ===============================================
    private final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // CLASS SPECIFIC VARIABLES ====================================================================
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
    double [] robotCoordinates = {0d, 0d};
    double rangeCoefficient = 0.0; // Used as average intermediate

    public AprilTagLocalizer(HardwareMap hardwareMap, Telemetry telemetry, double relativeX, double relativeY){
        // Match instance fields
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        initAprilTag();
        visionPortal.resumeStreaming(); // Assumes starting with no stream upon start of opmode

        // Wait for the DS start button to be touched
        this.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        this.telemetry.addData(">", "Touch Play to start OpMode");
        this.telemetry.update();

        // Class-specific variables
        CAMERA_RELATIVE_COORDINATES [0] = relativeX;
        CAMERA_RELATIVE_COORDINATES [1] = relativeY;
    }

    @Override
    double[] getCoords(double robotHeading, double currentX, double currentY) {
        return new double[0];
    }

    double [] getRelCoords(double robotHeading, double currentX, double currentY){ // Camera coordinates, relative to field; x, y, heading
        // Reset calculation variables
        rangeCoefficient = 0d;
        heading = 0d;
        cameraCoordinates [0] = 0d;
        cameraCoordinates [1] = 0d;

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(currentDetections.size() == 0 || currentDetections == null){
            return new double [] {currentX, currentY, robotHeading};
        }

        for(AprilTagDetection detection : currentDetections) { // Calculate range coefficient
            if(detection.id == 3 || detection.id == 4) {
                rangeCoefficient += 1d /
                        (detection.ftcPose.range * detection.ftcPose.range);
            }else {
                rangeCoefficient += 0.1 /
                        (detection.ftcPose.range * detection.ftcPose.range);
            }
        }

        for(AprilTagDetection detection : currentDetections) {
            if(detection.id == 3 || detection.id == 4) {
                heading += (detection.ftcPose.yaw * 1d) /
                        ((detection.ftcPose.range * detection.ftcPose.range));
            }else if(detection.id > 6 ) {

            }else {
                heading += (detection.ftcPose.yaw * 0.1) /
                        ((detection.ftcPose.range * detection.ftcPose.range));
            }
        }

        for(AprilTagDetection detection : currentDetections) {
            if(detection.id == 3 || detection.id == 4) {
                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * (detection.ftcPose.range * detection.ftcPose.range)
                        - APRIL_TAG_COORDS[detection.id - 1][0]) / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));

                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * (detection.ftcPose.range * detection.ftcPose.range)
                        - APRIL_TAG_COORDS[detection.id - 1][1]) / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else {
                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * (detection.ftcPose.range * detection.ftcPose.range)
                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));

                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * (detection.ftcPose.range * detection.ftcPose.range)
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

    // COPIED OVER FROM REFERENCE CLASS ============================================================

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true) // IDK WHY I DID THIS AHAHAHAHAHAHAH
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)

                // == CAMERA CALIBRATION == IDK THE SPECS FOR THE CAMERA
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void telemetryAprilTag() {

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        this.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                this.telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                this.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                this.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                this.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                this.telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                this.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        this.telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        this.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        this.telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}
