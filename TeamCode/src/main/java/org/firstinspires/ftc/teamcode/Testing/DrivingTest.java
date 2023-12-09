package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Camera.PixelDetector;
import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp(name = "DrivingTest")
public class DrivingTest extends OpMode {

    MecanumDrive mecanumDrive;
    Servo leftServo;
    Servo rightServo;
    DcMotorEx intakeMotor;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
//        rightServo = hardwareMap.get(Servo.class, "rightServo");
//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

    }

    @Override
    public void loop() {
//        if (gamepad1.x) {
//            leftServo.setPosition(0);
//        }
//
//        if (gamepad1.y) {
//            leftServo.setPosition(1);
//        }
//
//        if (gamepad1.a) {
//            rightServo.setPosition(0);
//        }
//        if (gamepad1.b) {
//            rightServo.setPosition(1);
//        }

//        telemetry.addData("Y: ", "Left servo 1");
//        telemetry.addData("X: ", "Left servo 0");
//        telemetry.addData("B: ", "Right servo 1");
//        telemetry.addData("A: ", "Right servo 0");
//
        mecanumDrive.normalDrive(1, -gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

}

