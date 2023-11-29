package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.OuttakeBox;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp (name = "Servo Test", group = "Testing")
public class OuttakeTest extends OpMode {

    OuttakeBox outtakeBox;
    Servo servo;

    ButtonToggle buttonToggleY;
    ButtonToggle buttonToggleA;

    @Override
    public void init() {
        buttonToggleY = new ButtonToggle();
        buttonToggleA = new ButtonToggle();
        outtakeBox = new OuttakeBox(hardwareMap, "outtake_servo");
        servo = hardwareMap.get(Servo.class, "outtake_servo");
    }

    double position = 0.25;
    double increment = 0.01;
    @Override
    public void loop() {
        boolean y = buttonToggleY.update(gamepad1.y);
        boolean a = buttonToggleA.update(gamepad1.a);

        if (y) {
            position += increment;
        }
        if (a) {
            position -= increment;
        }

        servo.setPosition(position);
        telemetry.addData("Position", Math.round(servo.getPosition() * 1000) / 1000.0);
        telemetry.addData("Direction", servo.getDirection());
    }


}
