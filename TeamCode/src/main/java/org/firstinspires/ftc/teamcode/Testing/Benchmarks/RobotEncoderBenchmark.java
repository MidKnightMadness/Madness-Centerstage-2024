package org.firstinspires.ftc.teamcode.Testing.Benchmarks;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.DeadReckoningDrive;
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


    final static String OUTPUT_FILE = "encoder.csv";

    DcMotor encoder1;
    DcMotor encoder2;
    DcMotor encoder3;
    Servo intakeServo;

    DeadReckoningDrive deadReckoningDrive;

    @Override
    public void init() {
        timer = new Timer();
        intakeServo = hardwareMap.get(Servo.class, "Right intake servo");
        intakeServo.setPosition(0.066);
        String filePath = String.format("%s/FIRST/data/%s",
                Environment.getExternalStorageDirectory().getAbsolutePath(), OUTPUT_FILE);

        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);

        try {
            fileWriter = new FileWriter(filePath);
            bufferedWriter = new BufferedWriter(fileWriter);
            bufferedWriter.write("Elapsed time,Encoder Position, Inches");

            telemetry.addLine("Successfully able to write to "+ OUTPUT_FILE);
        }
        catch (IOException e) {
            String errorMessage = e.getMessage();
            telemetry.addData("Error", errorMessage);
        }






    }

    @Override
    public void init_loop() {
        telemetry.addData("Power", power);
        power += gamepad1.left_stick_y * 0.001;

    }
    @Override
    public void start() {
        telemetry.clear();
        deadReckoningDrive.resetEncoders();

    }

    boolean stopped = false;
    double power = 0.3;
    @Override
    public void loop() {


        deadReckoningDrive.setPowers(power, power, power, power);

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
        deadReckoningDrive.updateDisplacement();
        writeData(timer.getTime(), deadReckoningDrive.getTicks().y, deadReckoningDrive.getDisplacement().y);
    }

    void writeData(double elapsedTime, double ticks, double displacement) {
        try {
            bufferedWriter.write(String.format("%f,%f,%f\n", elapsedTime, ticks, displacement));
        }
        catch (IOException e) {
            telemetry.addData("Error", e.getMessage());
        }
    }
}