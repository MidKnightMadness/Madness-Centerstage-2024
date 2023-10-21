package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagLocalizer extends Localizer{ // Currently runs on main thread, may be an issue, uses ConceptAprilTag as basis
    // AUXILLARY VARIABLES =========================================================================
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public double [] sensorCoords1 = {0.0, 0.0}; // For each tag detected, do both localization methods
    public double [] sensorCoords2 = {0.0, 0.0};

    // VARIABLES COPIED FROM CONCEPT APRIL TAG CLASS ===============================================
    private final boolean USE_WEBCAM = true;
    private AprilTagDetection aprilTagDetection;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // CLASS SPECIFIC VARIABLES ====================================================================
    private boolean running = false;
    public List <Vector2> sensorCoordiantesDetected;
    public double [][] aprilTagCoordinates = {
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0},
            {0.0, 0.0}
    };
    public double [] calculationsVector = {0.0, 0.0}; // Used as intermediate

    public AprilTagLocalizer (HardwareMap hardwareMap, Telemetry telemetry, double relX, double relY, double cameraAngle){
        // Match instance fields
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        initAprilTag();
        visionPortal.resumeStreaming(); // Assumes starts with no stream

        // Wait for the DS start button to be touched.
        this.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        this.telemetry.addData(">", "Touch Play to start OpMode");
        this.telemetry.update();

        this.running = true;
    }

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
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
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

    private void telemetryAprilTag() {

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

    private void toggle(){
        this.running = !this.running;
        if(running){
            visionPortal.stopStreaming();
        }else{
            visionPortal.resumeStreaming();
        }
    }

    @Override
    double[] getRelCoords(double robotHeading, double currentX, double currentY) {
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        this.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Reset list
        sensorCoordiantesDetected.clear();

        // Step through the list of detections and display info for each one.
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            if(detection.metadata != null){
                // METHOD 1 : USING RELATIVE X AND Y OF TAG TO CAMERA ==============================
                sensorCoords1 [0] =  detection.ftcPose.x; // When pointing at 0˚, y with respects to robot is x with resepcts to field, vice versa for x
                sensorCoords1 [1] =  detection.ftcPose.y;

                // METHOD 2 : USING BEARING AND RANGE ==============================================
                sensorCoords2 [0] = detection.ftcPose.range * Math.sin(detection.ftcPose.bearing);
                sensorCoords2 [1] = detection.ftcPose.range * Math.cos(detection.ftcPose.bearing);

                // Combine relative coordinates, coefficients not determined yet
                calculationsVector [0] = 0.5 * sensorCoords1[0] + 0.5 * sensorCoords2[0];
                calculationsVector [1] = 0.5 * sensorCoords1[1] + 0.5 * sensorCoords2[1];

                // Rotate negative of relative coordinates by -(heading - 90˚), add coordinates from this localization method
                sensorCoordiantesDetected.add(new Vector2(
                    -Math.cos(-(robotHeading - Math.PI / 2.0)) * this.calculationsVector[0] - Math.sin(-(robotHeading - Math.PI / 2.0)) * this.calculationsVector[1] + aprilTagCoordinates [detection.id - 1][0],
                        Math.sin(-(robotHeading - Math.PI / 2.0)) * this.calculationsVector[0] - Math.cos(-(robotHeading - Math.PI / 2.0)) * this.calculationsVector[1] + aprilTagCoordinates [detection.id - 1][1]
                ));
            }
        }

        return null;
    }

    @Override
    double[] getCoords(double robotHeading, double currentX, double currentY) {
        if(running){

            getRelCoords(robotHeading, currentX, currentY);

            // Combine detected coordinates with particle filter

        }

        return null;
    }
}
