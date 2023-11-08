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

public class AprilTagLocalizerTwo extends Localizer {
    // AUXILLARY VARIABLES =========================================================================
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    public double[] sensorCoords1 = {0.0, 0.0}; // For each tag detected, do both localization methods
    public double[] sensorCoords2 = {0.0, 0.0};

    // VARIABLES COPIED FROM CONCEPT APRIL TAG CLASS ===============================================
    private final boolean USE_WEBCAM = true;
    private AprilTagDetection aprilTagDetection;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // CLASS SPECIFIC VARIABLES ====================================================================
    private boolean running = false;
    public ArrayList<Vector2> sensorCoordinates;
    public ArrayList<Double> rangeCoefficients;
    double [] calculationsVector = {0.0, 0.0};

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

    public double[][] calculations;// Used as intermediate
    public double calculationsDouble = 0.0; // Used as intermediate
    double yAfterErrorChange = 0;
    double xAfterErrorChange = 0;

    @Override
    double[] getRelCoords(double robotHeading, double currentX, double currentY) {
        return new double[0];
    }

    @Override
    double[] getCoords(double robotHeading, double currentX, double currentY) {
        return new double[0];
    }

    private double correctY(double percievedY){
        return (percievedY + 1.200) / 1.068;
    }

    private double correctX(double percievedX, double actualY){
        return 0;
    }
}
