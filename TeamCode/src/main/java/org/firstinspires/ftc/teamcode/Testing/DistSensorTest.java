package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

@TeleOp(name = "Dist Sensor Test", group = "Sensor")
@Disabled
public class DistSensorTest extends OpMode {
    public DistanceSensor distanceSensor;
    AverageBuffer timeBuffer;

    Timer timer;
    @Override
    public void init() {
        timer = new Timer();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        timeBuffer = new AverageBuffer(10);
    }

    @Override
    public void loop() {
        timer.updateTime();

        timeBuffer.update(timer.getDeltaTime());
        double deltaTime = timeBuffer.getValue();

        telemetry.addLine(String.format("Update rate: %.3f Hz", 1.0 / deltaTime));
        log(distanceSensor);

        telemetry.addLine("--------------");
        telemetry.update();
    }

    public void log(DistanceSensor input) {
        telemetry.addLine(String.format("Name: %s", input.getDeviceName()));
        telemetry.addLine(String.format("Connection: %s", input.getConnectionInfo()));
        telemetry.addLine(String.format("Distance (mm): %.3f", input.getDistance(DistanceUnit.MM)));
    }
}