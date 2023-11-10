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

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalizerTwo extends Localizer {
    // AUXILLARY VARIABLES =========================================================================
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public double[] sensorCoords1 = {0.0, 0.0}; // For each tag detected, do both localization methods
    public double[] sensorCoords2 = {0.0, 0.0};
    double [] calculationsVector = {0.0, 0.0};

    // VARIABLES COPIED FROM CONCEPT APRIL TAG CLASS ===============================================
    private final boolean USE_WEBCAM = true;
    private AprilTagDetection aprilTagDetection;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // CLASS SPECIFIC VARIABLES ====================================================================
    public ArrayList<Vector2> sensorCoordinates;
    public ArrayList<Double> rangeCoefficients;

    public final double[][] APRIL_TAG_COORDS = { // hardcoded
            {135, 114.75},//id 1
            {135, 108.75},//id 2
            {135, 102.75},//id 3
            {135, 41.25},//id 4
            {135, 35.25},//id 5
            {135, 29.25},//id 6
            {0.0, 114},//id 7 not necesarilky accurate yet
            {0.0, 108},//id 8 not necesarilky accurate yet
            {0.0, 36},//id 9 not necesarilky accurate yet
            {0.0, 30}//id 10 not necesarilky accurate y
    };
    public double calculationsDouble = 0.0; // Used as intermediate
    double yAfterErrorChange = 0;
    double xAfterErrorChange = 0;

    public AprilTagLocalizerTwo(HardwareMap hardwareMap, Telemetry telemetry, double relativeX, double relativeY){
        // Match instance fields
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        sensorCoordinates = new ArrayList<Vector2>();
        rangeCoefficients = new ArrayList<Double>();

        initAprilTag();
        visionPortal.resumeStreaming(); // Assumes starting with no stream upon start of opmode

        // Wait for the DS start button to be touched
        this.telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        this.telemetry.addData(">", "Touch Play to start OpMode");
        this.telemetry.update();
    }

    @Override
    public double[] getRelCoords(double robotHeading, double currentX, double currentY) {
        // Reset list of detected coordinates relative to camera
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        this.telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Reset lists
        sensorCoordinates.clear();
        rangeCoefficients.clear();
        calculationsVector [0] = 0.0;
        calculationsVector [1] = 0.0;
        calculationsDouble = 0.0;

        // Get range coefficients normalizing factor
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                calculationsDouble += 1 / (detection.ftcPose.range * detection.ftcPose.range); // Add up coefficients, divide everything by sum to normalize
            }
        }

        // Step through the list of detections and combine coordinates from each one
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addData("Detection", detection.id);

                // Intermediates
                double correctedY = correctY(detection.ftcPose.y);
                double correctedX = correctX(detection.ftcPose.x, correctY(detection.ftcPose.y));

                // Combine detection coordinates with inverse square coefficients based on range
                calculationsVector [0] -= Math.cos(robotHeading) * correctedX + Math.sin(robotHeading) * correctedY // Rotate to correct for robot heading
                                            -APRIL_TAG_COORDS [detection.id - 1][0];
                calculationsVector [1] -= Math.sin(robotHeading) * correctedX + Math.cos(robotHeading) * correctedY // Rotate to correct for robot heading
                                            -APRIL_TAG_COORDS [detection.id - 1][1];

                calculationsVector [0] /= (detection.ftcPose.range * detection.ftcPose.range * calculationsDouble);
                calculationsVector [1] /= (detection.ftcPose.range * detection.ftcPose.range * calculationsDouble);
            }
        }

        if(currentDetections.size() == 0){
            return null;
        }else{
            return calculationsVector;
        }
    }

    @Override
    double[] getCoords(double robotHeading, double currentX, double currentY) {
        return new double[0];
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
