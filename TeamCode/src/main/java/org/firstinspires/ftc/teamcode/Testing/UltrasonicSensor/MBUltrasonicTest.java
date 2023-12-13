package org.firstinspires.ftc.teamcode.Testing.UltrasonicSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Components.MBUltrasonicSensorWrapper;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

@TeleOp(name = "Ultrasonic Sensor Test", group = "Testing")
public class MBUltrasonicTest extends OpMode {
    AnalogInput us1;
    MBUltrasonicSensorWrapper ultrasonicSensor;

    AverageBuffer timeBuffer;
    AverageBuffer[] voltageBuffers = { new AverageBuffer(10), new AverageBuffer(10),
            new AverageBuffer(10), new AverageBuffer(10)
    };

    Timer timer;
    @Override
    public void init() {
        timer = new Timer();
        us1 = hardwareMap.get(AnalogInput.class, "us1"); // analog port 0
        ultrasonicSensor = new MBUltrasonicSensorWrapper(us1, 10);

        timeBuffer = new AverageBuffer(10);
    }

    @Override
    public void loop() {
        timer.updateTime();

        timeBuffer.update(timer.getDeltaTime());
        double deltaTime = timeBuffer.getValue();

        telemetry.addLine(String.format("Update rate: %.3f Hz", 1.0 / deltaTime));

        voltageBuffers[0].update(us1.getVoltage());
        double voltage = voltageBuffers[0].getValue();
        log(us1, voltage);

        double distance = ultrasonicSensor.update();
        telemetry.addData("Distance (mm)", distance);


        telemetry.addLine("--------------");
        telemetry.update();
    }

    void log(AnalogInput input, double voltage) {
        telemetry.addLine(String.format("Connection: %s", input.getConnectionInfo()));
        telemetry.addLine(String.format("Voltage: %.3f", voltage));
        // telemetry.addLine(String.format("Distance (mm): %.3f", voltageToDistance(input.getVoltage())));
    }

    double voltageToDistance(double voltage) {
        return ( voltage / (3.3/1024) ) * 6 - 300;
    }


}