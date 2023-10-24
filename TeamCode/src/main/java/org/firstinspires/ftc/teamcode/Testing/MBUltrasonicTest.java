package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

@TeleOp(name = "Ultrasonic Sensor Test")
public class MBUltrasonicTest extends OpMode {
    public AnalogInput ultrasonicSensor;
    public AnalogInput us2;
    AverageBuffer timeBuffer;

    Timer timer;
    @Override
    public void init() {
        timer = new Timer();
        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "ultrasonic_sensor");
        us2 = hardwareMap.get(AnalogInput.class, "us2");
        timeBuffer = new AverageBuffer(10);
    }

    @Override
    public void loop() {
        timer.updateTime();

        timeBuffer.update(timer.getDeltaTime());
        double deltaTime = timeBuffer.getValue();

        telemetry.addLine(String.format("Update rate: %.3f Hz", 1.0 / deltaTime));

        log(ultrasonicSensor);
        telemetry.addLine("--------------");
        log(us2);
        telemetry.update();
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