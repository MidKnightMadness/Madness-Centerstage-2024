package org.firstinspires.ftc.teamcode.Testing.Benchmarks;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;

@TeleOp(name = "Robot Encoder Benchmark", group = "Benchmark")
public class RobotEncoderBenchmark extends OpMode {
    AverageBuffer timeBuffer;
    BufferedWriter bufferedWriter;
    FileWriter fileWriter;
    Timer timer;

    final int NUM_DATAPOINTS = 5000;
    int datapoints = 0;

    Vector2<Double> cameraResolution = new Vector2<Double>(800, 600);

    final static String OUTPUT_FILE = "benchmark.csv";

    DcMotor encoder1;
    DcMotor encoder2;
    DcMotor encoder3;

    @Override
    public void init() {
        encoder1 = hardwareMap.get(DcMotor.class, "encoder1");
        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2 = hardwareMap.get(DcMotor.class, "encoder2");
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder3 = hardwareMap.get(DcMotor.class, "encoder3");
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new Timer();
        timeBuffer = new AverageBuffer(1);
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
            log(encoder1);
            log(encoder2);
            log(encoder3);
            log();
            datapoints++;
        }
    }
    void log(DcMotor encoder) {
        telemetry.addLine(String.format("Connection: %s", encoder.getConnectionInfo()));
        telemetry.addLine(String.format("Position: %d", encoder.getCurrentPosition()));
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