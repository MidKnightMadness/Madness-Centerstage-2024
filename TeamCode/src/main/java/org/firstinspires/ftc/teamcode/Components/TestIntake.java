package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intake testing")
public class TestIntake extends OpMode {
    Servo rightServo;

    CRServo leftServo;
//    Servo leftServo;

    @Override
    public void init() {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

        leftServo.setDirection(CRServo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("Right servo", "y - a");
        telemetry.addData("Left servo", "x - b");

        if (gamepad1.a) {
            rightServo.setPosition(0);
        }
        if (gamepad1.x) {
            leftServo.setPower(0);
        }
        if (gamepad1.b) {
            leftServo.setPower(0.5);
        }
        if (gamepad1.y) {
            rightServo.setPosition(1);
        }

        telemetry.addData("Left servo", leftServo.getPower());
        telemetry.addData("Right servo", rightServo.getPosition());

    }
}
