package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.PixelDetector;
import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.SectionSpline;
import org.firstinspires.ftc.teamcode.Drivetrain.SplinePath;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Components.LinearSlides;
import org.firstinspires.ftc.teamcode.Components.OuttakeBox;

@TeleOp(group= "[Game]", name = "Driver Controlled TeleOp")
public class Main extends OpMode {
    MecanumDrive mecanumDrive;

    DcMotor intakeMotor;
    OuttakeBox OuttakeServo;

    ButtonToggle buttonToggleA, g1RightBump;

    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;

    Servo rightIntakeServo;
    Servo leftIntakeServo, boxServo, rightElbowServo, rightWristServo;


    ButtonToggle g2Y, g2A, g2LeftBump;

    DcMotorEx IntakeMotor;

    double[] rightIntakeServoPositions = {0.3075, 0.2425, 0.2223, 0.1605, 0.126};
    double[] leftIntakeServoPositions = {0.8375, 0.89, 0.893, 0.939, 0.9575};

    double wristVertical = 0.623;
    double wristDown = 0.388;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake motor");

        rightElbowServo = hardwareMap.get(Servo.class, "Right elbow servo");

        g2Y = new ButtonToggle();
        g2A = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        buttonToggleA = new ButtonToggle();
        g1RightBump = new ButtonToggle();

        rightIntakeServo = hardwareMap.get(Servo.class, "Right intake servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");

        boxServo = hardwareMap.get(Servo.class, "Center box servo");

        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");
        buttonToggleA = new ButtonToggle();

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized");
    }

    double power = 1;
    public void handleDriverControls() {
        if (g1RightBump.update(gamepad1.right_bumper)) {
            if (power == 1) {
                power = 0.25;
            }
            else {
                power = 1;
            }
        }
        mecanumDrive.normalDrive(power, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }

    int currentIntakeServoIndex = 0;

    double wristPos = wristDown;
    public void handleManipulatorControls() {

        if (g2LeftBump.update(gamepad2.left_bumper)) {
            if (wristPos == wristDown) {
                wristPos = wristVertical;
            }
            else {
                wristPos = wristDown;
            }
        }
        if (g2Y.update(gamepad2.y)) {
            if (currentIntakeServoIndex < rightIntakeServoPositions.length - 1) {
                currentIntakeServoIndex++;
            }
        }

        if (g2A.update(gamepad2.a)) {
            if (currentIntakeServoIndex > 0) {
                currentIntakeServoIndex--;
            }
        }

        if (this.gamepad2.right_bumper) {
            boxServo.setPosition(1);
        }
        else {
            boxServo.setPosition(0.69);
        }

        rightWristServo.setPosition(wristPos);



        telemetry.addData("Intake servo pos index", currentIntakeServoIndex);
        telemetry.addData("Right intake servo", rightIntakeServo.getPosition());
        telemetry.addData("Left intake servo", leftIntakeServo.getPosition());

        rightIntakeServo.setPosition(rightIntakeServoPositions[currentIntakeServoIndex]);
        intakeMotor.setPower(gamepad2.left_stick_y);
        motorLeft.setPower(this.gamepad2.right_stick_y * -0.5);
        motorRight.setPower(this.gamepad2.right_stick_y * -0.5);
    }

    @Override
    public void loop() {
        handleDriverControls();
        handleManipulatorControls();
    }
}

