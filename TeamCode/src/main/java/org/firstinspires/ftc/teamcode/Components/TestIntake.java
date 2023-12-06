package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "intake testing")
public class TestIntake extends OpMode {
    Servo rightServo;

    @Override
    public void init() {
        rightServo = hardwareMap.get(Servo.class, "rightServo");
    }

    @Override
    public void loop() {

    }
}
