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

@TeleOp(name = "DrivingTest", group = "testing")
public class DrivingTest extends OpMode {

    MecanumDrive mecanumDrive;
    Servo leftServo;
    Servo rightServo;
    DcMotorEx intakeMotor;

    double increment = 0.0005;
    double rotationPower = 0.1;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
//        rightServo = hardwareMap.get(Servo.class, "rightServo");
//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");


    }

    @Override
    public void loop() {
        if (this.gamepad1.dpad_up) {
            rotationPower += increment;
        }

        if (gamepad1.dpad_down) {
            rotationPower -= increment;
        }

        if (this.gamepad1.right_bumper) {
            mecanumDrive.normalDrive(rotationPower, 0, 0, 1);
        }

        if (this.gamepad1.left_bumper) {
            mecanumDrive.normalDrive(rotationPower, 0, 1, 0);
        }

        telemetry.addData("Adjust Power", rotationPower);

        mecanumDrive.normalDrive(1, -gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }

}

