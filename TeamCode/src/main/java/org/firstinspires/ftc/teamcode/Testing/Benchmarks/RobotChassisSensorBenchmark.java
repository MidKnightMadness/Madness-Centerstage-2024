package org.firstinspires.ftc.teamcode.Testing.Benchmarks;

import android.os.Environment;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.RGBColor;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.List;

@TeleOp(name = "Robot Chassis Sensor Benchmark", group = "Benchmark")
@Disabled
public class RobotChassisSensorBenchmark extends OpMode {
    AverageBuffer timeBuffer;
    BufferedWriter bufferedWriter;
    FileWriter fileWriter;
    Timer timer;


    final int NUM_DATAPOINTS = 5000;
    int datapoints = 0;

    Vector2 cameraResolution = new Vector2(800, 600);

    final static String OUTPUT_FILE = "benchmark.csv";

    // camera initialization
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    ColorSensor cs1;
    ColorSensor cs2;

    AnalogInput us1;
    AnalogInput us2;

    DcMotor encoder1;
    DcMotor encoder2;
    DcMotor encoder3;

    @Override
    public void init() {
        telemetry.setAutoClear(true);
        timer = new Timer();
        timeBuffer = new AverageBuffer(1);
        initAprilTag();

        cs1 = hardwareMap.get(ColorSensor.class, "cs1");
        cs2 = hardwareMap.get(ColorSensor.class, "cs2");

        us1 = hardwareMap.get(AnalogInput.class, "us1");
        us2 = hardwareMap.get(AnalogInput.class, "us2");

        encoder1 = hardwareMap.get(DcMotor.class, "encoder1");
        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2 = hardwareMap.get(DcMotor.class, "encoder2");
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder3 = hardwareMap.get(DcMotor.class, "encoder3");
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        String filePath = String.format("%s/FIRST/data/%s",
                Environment.getExternalStorageDirectory().getAbsolutePath(), OUTPUT_FILE);

        try {
            fileWriter = new FileWriter(filePath);
            bufferedWriter = new BufferedWriter(fileWriter);
            bufferedWriter.write("Elapsed time,FPS,Delta time,pipeline,overhead\n");

            telemetry.addLine("Successfully able to write to "+ OUTPUT_FILE);
        }
        catch (IOException e) {
            String errorMessage = e.getMessage();
            telemetry.addData("Error", errorMessage);
        }

    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size((int) cameraResolution.x, (int) cameraResolution.y));

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
    }

    @Override

    public void start() {
        telemetry.clear();

        aprilTag.getPerTagAvgPoseSolveTime();
    }

    boolean stopped = false;
    @Override
    public void loop() {
        if (datapoints == NUM_DATAPOINTS) {
            if (!stopped) {
                stop();
                stopped = true;
            }
            else {
                telemetry.addLine("Stop robot");
            }
        }
        else {
            log(cs1);
            log(cs2);
            log(us1);
            log(us2);
            log(encoder1);
            log(encoder2);
            log(encoder3);
            telemetryAprilTag();
            log();
            datapoints ++;
        }
    }

    @Override
    public void stop() {
        try {
            bufferedWriter.flush();
            bufferedWriter.close();
            fileWriter.close();
        }
        catch (IOException e) { telemetry.addLine(e.getMessage()); }
    }

    void log() {
        timer.updateTime();
        writeData(timer.getTime(), timer.getDeltaTime(), 1.0 / timer.getDeltaTime(), 0, 0);
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }

    void log(ColorSensor in) {
        RGBColor reading = new RGBColor(in.red(), in.green(), in.blue(), in.alpha());
        telemetry.addData("RGBA", reading);
        telemetry.addLine("-------------------------------");
    }

    void log(AnalogInput in) {
        telemetry.addData("Voltage:", in.getVoltage());
        telemetry.addLine("-------------------------------");
    }

    void log(DcMotor encoder) {
        telemetry.addLine(String.format("Connection: %s", encoder.getConnectionInfo()));
        telemetry.addLine(String.format("Position: %d", encoder.getCurrentPosition()));
        telemetry.addLine("-------------------------------");
    }


    void writeData(double elapsedTime, double deltaTime, double fps, double pipeline, double overhead) {
        try {
            bufferedWriter.write(String.format("%f,%f,%f,%f,%f\n", elapsedTime, fps, deltaTime, pipeline, overhead));
        }
        catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
    }
}