package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Ultrasonic Sensor Test")
public class MBUltrasonicTest extends OpMode {
    public AnalogInput ultrasonicSensor;

    @Override
    public void init() {
        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "ultrasonic_sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Voltage", ultrasonicSensor.getVoltage());
        telemetry.addData("Distance (mm)", ultrasonicSensor.getVoltage() * 5d);
    }
}