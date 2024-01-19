package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp(group= "aGame", name = "Driver Controlled TeleOp")
public class Main extends OpMode implements ServoPositions {
    MecanumDrive mecanumDrive;
    DcMotor intakeMotor;
    ButtonToggle g1A, g1RightBump;
    public DcMotorEx motorRight, motorLeft;
    Servo rightIntakeServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    IMU imu;
    ButtonToggle g2Y, g2A, g2LeftBump, g2RightBump, g2X;
    boolean isIntakeMode = true;

    Servo launcherServo;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake motor");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        launcherServo = hardwareMap.get(Servo.class, "Launcher servo");
        init_IMU();

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
        if (g1RightBump.update(gamepad1.right_bumper)) {
            gamepad1.rumble(300);
            if (power == 1) {
                power = 0.35;
            }
            else {
                power = 1;
            }
        }

//        mecanumDrive.normalDrive(power, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
        mecanumDrive.FieldOrientedDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), telemetry);
        telemetry.addData("Yaw, ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        telemetry.update();

        if(gamepad1.x){
            imu.resetYaw();
        }
    }

    @Override
    public void loop() {
        handleDriverControls();
        handleManipulatorControls();

        telemetry();
    }

    double wristPos = wristServoIn;
    public void handleManipulatorControls() {
        handleIntakeControls();
        handleOuttakeControls();

        if (gamepad2.left_stick_x > 0.1 || gamepad2.right_stick_x > 0.1 || gamepad2.left_stick_y > 0.1) {
            mecanumDrive.normalDrive(power, -gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x);
        }

        // launcher
        if (gamepad2.dpad_up && gamepad2.y) {
            launcherServo.setPosition(launcherOpen);
        }
        else {
            launcherServo.setPosition(launcherClosed);
        }
    }

    void handleIntakeControls() {
        double intakeDirection = gamepad2.a ? 1 : -1;
        intakeMotor.setPower(gamepad2.left_trigger * intakeDirection);

        if (gamepad2.y) {
            rightIntakeServo.setPosition(intakeLowest + (intakeHighest - intakeLowest) * gamepad2.left_stick_y);;
        }
        else {
            rightIntakeServo.setPosition(intakeDefault);
        }
    }

    void handleOuttakeControls() {
        double slidesDirection = gamepad2.a ? -1 : 1;
        motorLeft.setPower(slidesDirection * gamepad2.right_trigger);
        motorRight.setPower(slidesDirection * gamepad2.right_trigger);

        if (this.gamepad2.right_bumper) {
            if (gamepad2.a) {
                boxServo.setPosition(boxServoRight);  // right
            } else {
                boxServo.setPosition(boxServoLeft);  // left
            }
        } else {
            boxServo.setPosition(boxServoNeutral);   // center
        }

        if (g2X.update(gamepad2.x)) {
            if (wristPos == wristServoIn) { wristPos = wristServoOut; }
            else { wristPos = wristServoIn; }
            rightWristServo.setPosition(wristPos);
        }
    }

    void telemetry() {
        telemetry.addData("Mode", isIntakeMode ? "Intake" : "Outtake");
    }

    void init_IMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }
}

