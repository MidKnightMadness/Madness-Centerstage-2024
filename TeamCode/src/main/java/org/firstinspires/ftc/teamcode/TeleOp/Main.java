package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Components.ServoPositions.wristServoIn;
import static org.firstinspires.ftc.teamcode.Components.ServoPositions.wristServoOut;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp(group= "[Game]", name = "Driver Controlled TeleOp")
public class Main extends OpMode {
    MecanumDrive mecanumDrive;
    DcMotor intakeMotor;
    ButtonToggle g1A, g1RightBump;
    public DcMotorEx motorRight, motorLeft;
    Servo rightIntakeServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    ButtonToggle g2Y, g2A, g2LeftBump, g2RightBump, g2X;
    double[] rightIntakeServoPositions = {0.3075, 0.2425, 0.2223, 0.1605, 0.126};
    double[] leftIntakeServoPositions = {0.8375, 0.89, 0.893, 0.939, 0.9575};

    boolean isIntakeMode = true;
    ElapsedTime timer;
    double updateRate = 0.0;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake motor");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        timer = new ElapsedTime();

        rightElbowServo = hardwareMap.get(Servo.class, "Right elbow servo");
                g2Y = new ButtonToggle();
        g2A = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        g1A = new ButtonToggle();
        g1RightBump = new ButtonToggle();
        g2X = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        rightIntakeServo = hardwareMap.get(Servo.class, "Right intake servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");

        boxServo = hardwareMap.get(Servo.class, "Center box servo");

        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized");
    }

    double power = 1;
    public void handleDriverControls() {
//        if (g1RightBump.update(gamepad1.right_bumper)) {
//            if (power == 1) {
//                power = 0.25;
//            }
//            else {
//                power = 1;
//            }
//        }
        mecanumDrive.normalDrive(power, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        timer.reset();
        updateRate = 1d / timer.milliseconds();
    }

    int currentIntakeServoIndex = 0;

    double wristPos = wristServoIn;
    public void handleManipulatorControls() {
        if (g2LeftBump.update(gamepad2.left_bumper)) {
            gamepad2.rumble(1000);
            isIntakeMode = !isIntakeMode;
        }

        if (isIntakeMode) {
            handleIntakeControls();
        }
        else {
            handleOuttakeControls();
        }
//        if (g2Y.update(gamepad2.y)) {
//            if (currentIntakeServoIndex < rightIntakeServoPositions.length - 1) {
//                currentIntakeServoIndex++;
//            }
//        }
//
//        if (g2A.update(gamepad2.a)) {
//            if (currentIntakeServoIndex > 0) {
//                currentIntakeServoIndex--;
//            }
//        }
    }

    @Override
    public void loop() {
        handleDriverControls();
        handleManipulatorControls();
        telemetry();
    }

    void handleIntakeControls() {
        intakeMotor.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        if (gamepad2.a) {
            rightIntakeServo.setPosition(0.126 + (0.3075 - 0.126) * gamepad2.right_trigger);;
        }
        else {
            rightIntakeServo.setPosition(0.185);
        }
    }

    void handleOuttakeControls() {
        if(gamepad2.left_stick_y < 0){
            motorLeft.setPower(-this.gamepad2.left_stick_y * 1);
            motorRight.setPower(-this.gamepad2.left_stick_y * 1);
        }else{
            motorLeft.setPower(this.gamepad2.left_stick_y * 0.5);
            motorRight.setPower(this.gamepad2.left_stick_y * 0.5);
        }

        if (this.gamepad2.right_bumper) {
            if (gamepad2.a) {
                boxServo.setPosition(0.45);  // right
            } else {
                boxServo.setPosition(0.85);  // left
            }
        } else {
            boxServo.setPosition(0.6435);   // center
        }

        if (g2X.update(gamepad2.x)) {
            if (wristPos == wristServoIn) {wristPos = wristServoOut; }
            else { wristPos = wristServoIn; }
            rightWristServo.setPosition(wristPos);
        }
    }

    void telemetry() {
        telemetry.addData("Mode", isIntakeMode ? "Intake" : "Outtake");

    }
}

