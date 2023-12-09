package org.firstinspires.ftc.teamcode.Testing.Benchmarks;

import android.os.Environment;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name = "Robot Ultrasonic Benchmark", group = "Benchmark")
@Disabled
public class RobotUSBenchmark extends OpMode {
    BufferedWriter bufferedWriter;
    FileWriter fileWriter;
    Timer timer;

    final int NUM_DATAPOINTS = 5000;
    int datapoints = 0;

    final static String OUTPUT_FILE = "benchmark.csv";

    AnalogInput us1;
    AnalogInput us2;

    @Override
    public void init() {
        us1 = hardwareMap.get(AnalogInput.class, "us1");
        us2 = hardwareMap.get(AnalogInput.class, "us2");

        timer = new Timer();

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

    @Override
    public void start() {
        telemetry.clear();
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
            log(us1);
            log(us2);
            log();
            datapoints++;
        }
    }
    void log(AnalogInput in) {
        telemetry.addData("Voltage:", in.getVoltage());
        telemetry.addLine("-------------------------------");
    }


    @Override
    public void stop() {
        try {
            bufferedWriter.flush();
            bufferedWriter.close();
            fileWriter.close();
        }
        catch (IOException e) { }
    }

    void log() {
        timer.updateTime();
        writeData(timer.getTime(), timer.getDeltaTime(), 1.0 / timer.getDeltaTime());
    }

    void writeData(double elapsedTime, double deltaTime, double fps) {
        try {
            bufferedWriter.write(String.format("%f,%f,%f\n", elapsedTime, fps, deltaTime));
        }
        catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
    }
}