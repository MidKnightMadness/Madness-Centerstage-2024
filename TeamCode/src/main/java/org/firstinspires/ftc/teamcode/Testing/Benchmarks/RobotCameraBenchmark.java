package org.firstinspires.ftc.teamcode.Testing.Benchmarks;

import android.os.Environment;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;

@TeleOp(name = "Robot Camera Benchmark", group = "Benchmark")
public class RobotCameraBenchmark extends OpMode {
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


    @Override
    public void init() {
        telemetry.setAutoClear(false);
        timer = new Timer();
        timeBuffer = new AverageBuffer(1);
        initAprilTag();
        // file initialization
//        String directory_path = Environment.getExternalStorageDirectory().getPath()+"/"+this.FOLDER;

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

    void writeData(double elapsedTime, double deltaTime, double fps, double pipeline, double overhead) {
        try {
            bufferedWriter.write(String.format("%f,%f,%f,%f,%f\n", elapsedTime, fps, deltaTime, pipeline, overhead));
        }
        catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
    }
}