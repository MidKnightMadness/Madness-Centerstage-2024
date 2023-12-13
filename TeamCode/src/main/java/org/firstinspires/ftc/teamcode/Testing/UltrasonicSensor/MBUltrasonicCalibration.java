package org.firstinspires.ftc.teamcode.Testing.UltrasonicSensor;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.IOException;
import java.util.Arrays;

@TeleOp(name = "Ultrasonic Sensor Calibration", group = "testing")
@Disabled
public class MBUltrasonicCalibration extends OpMode {
    public AnalogInput ultrasonicSensor;
    AverageBuffer timeBuffer;

    BufferedWriter bufferedWriter;
    FileWriter fileWriter;
    Timer timer;

    final double samplesPerDistance = 100;
    final long SLEEP_TIME_PER_SAMPLE = 10;
    boolean useTelemetry = false;
    final static String OUTPUT_FILE = "ultrasonic.csv";

    double[] distances = {1100, 1200, 1300, 1400, 1500};

    @Override
    public void init() {
        telemetry.setAutoClear(false);
        timer = new Timer();
        timeBuffer = new AverageBuffer(1);

        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "us1");

        // file initialization
//        String directory_path = Environment.getExternalStorageDirectory().getPath()+"/"+this.FOLDER;

        String filePath = String.format("%s/FIRST/data/%s",
                Environment.getExternalStorageDirectory().getAbsolutePath(), OUTPUT_FILE);

        try {
            fileWriter = new FileWriter(filePath);
            bufferedWriter = new BufferedWriter(fileWriter);
            bufferedWriter.write("Elapsed time,Delta time,Voltage,Distance\n");

            telemetry.addLine("Successfully able to write to "+ OUTPUT_FILE);
        }
        catch (IOException e) {
           String errorMessage = e.getMessage();
           telemetry.addData("Error", errorMessage);
        }

        telemetry.addLine("The OpMode will read at distances " + Arrays.toString(distances));

    }

    @Override
    public void init_loop() {
        telemetry.addData("Voltage", ultrasonicSensor.getVoltage());
    }

    int loggingIndex = 0;
    boolean firstIteration = true;
    @Override

    public void start() {
        telemetry.clear();
    }

    @Override
    public void loop() {
        if (firstIteration && loggingIndex <= distances.length - 1) {
            telemetry.addData("Logging at distance", distances[loggingIndex] + "mm");
            telemetry.addLine("Press X to continue");
            firstIteration = false;
        }

        if (this.gamepad1.x) {
            logAtDistance(distances[loggingIndex]);
        }

        if (loggingIndex == (distances.length)) {
            telemetry.addLine("Data collection complete");
            try {
                bufferedWriter.close();
                fileWriter.close();
            }
            catch (IOException e) { }

            firstIteration = false;
            telemetry.update();
        }
    }

    @Override
    public void stop() {

    }

    void logAtDistance(double distance) {
        firstIteration = true;
        loggingIndex++;

        timer.updateTime();

        for (int i = 0; i < samplesPerDistance; i++) {
            timer.updateTime();

            timeBuffer.update(timer.getDeltaTime());

            double deltaTime = timeBuffer.getValue();
            double elapsedTime = timer.getTime();

            if (useTelemetry) {
                telemetry.addLine(String.format("Update rate: %.3f Hz", 1.0 / deltaTime));
                telemetry.addLine("--------------------");
                log(ultrasonicSensor);
                telemetry.update();
            }

            writeData(elapsedTime, deltaTime, ultrasonicSensor.getVoltage(), distance);

            try {
                Thread.sleep(SLEEP_TIME_PER_SAMPLE);
            }
            catch (InterruptedException e) {
                telemetry.addLine("Error sleeping");
            }
        }
        
        telemetry.addLine("Successfully logged at distance " + distance + "mm\n");
        telemetry.addLine("------------------");
        telemetry.update();
    }

    void writeData(double elapsedTime, double deltaTime, double voltageReadings, double distance) {
        try {
            bufferedWriter.write(String.format("%f,%f,%f,%f\n", elapsedTime, deltaTime, voltageReadings, distance));
        }
        catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
    }

     void log(AnalogInput input) {
        telemetry.addLine(String.format("Name: %s", input.getDeviceName()));
        telemetry.addLine(String.format("Connection: %s", input.getConnectionInfo()));
        telemetry.addLine(String.format("Max voltage: %s", input.getMaxVoltage()));
        telemetry.addLine(String.format("Voltage: %.3f", input.getVoltage()));
        telemetry.addLine(String.format("Distance (mm): %.3f", voltageToDistance(input.getVoltage())));
    }

    double voltageToDistance(double voltage) {
        return ( voltage / (3.3/1024) ) * 6 - 300;
    }
}