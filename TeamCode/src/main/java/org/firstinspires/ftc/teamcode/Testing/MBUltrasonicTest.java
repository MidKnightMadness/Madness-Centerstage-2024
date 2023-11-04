package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

@TeleOp(name = "Ultrasonic Sensor Test")
public class MBUltrasonicTest extends OpMode {
    AnalogInput us1;
    AnalogInput us2;
    AnalogInput us3;
    AnalogInput us4;
    AverageBuffer timeBuffer;
    AverageBuffer[] voltageBuffers = { new AverageBuffer(10), new AverageBuffer(10),
            new AverageBuffer(10), new AverageBuffer(10)
    };

    Timer timer;
    @Override
    public void init() {
        timer = new Timer();
        us1 = hardwareMap.get(AnalogInput.class, "us1"); // analog port 0
        us2 = hardwareMap.get(AnalogInput.class, "us2"); // analog port 1
        us3 = hardwareMap.get(AnalogInput.class, "us3"); // analog port 2
        us4 = hardwareMap.get(AnalogInput.class, "us4"); // analog port 3

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

        voltageBuffers[1].update(us2.getVoltage());
        log(us2, voltageBuffers[1].getValue());

        voltageBuffers[2].update(us3.getVoltage());
        log(us3, voltageBuffers[2].getValue());

        voltageBuffers[3].update(us4.getVoltage());
        log(us4, voltageBuffers[3].getValue());


        telemetry.addLine("--------------");
        telemetry.update();
    }

    void log(AnalogInput input, double voltage) {
        telemetry.addLine(String.format("Connection: %s", input.getConnectionInfo()));
        telemetry.addLine(String.format("Voltage: %.3f", voltage));
        telemetry.addLine(String.format("Distance (mm): %.3f", voltageToDistance(input.getVoltage())));
    }

    double voltageToDistance(double voltage) {
        return ( voltage / (3.3/1024) ) * 6 - 300;
    }


}