package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;
//import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;

@TeleOp(group= "aGame", name = "Driver Controlled TeleOp")
public class Main extends OpMode implements ServoPositions {
    MecanumDrive mecanumDrive;
    DcMotor intakeMotor;
    ButtonToggle g1A, g1RightBump, g1LeftBump;
    public DcMotorEx motorRight, motorLeft;
    Servo rightIntakeServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    ButtonToggle g2Y, g2A, g2LeftBump, g2RightBump, g2X;
    boolean isIntakeMode = true;
    IMU imu;
    ModernRoboticsI2cRangeSensor rangeSensor;
    WebcamName webcamName;
    AprilTagLocalizerTwo localizer;

    double rotationResetConstant = Math.PI / 2;

    Servo launcherServo;
    boolean isUsingFieldOriented;
    ServoSmooth boxServoController;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake motor");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        launcherServo = hardwareMap.get(Servo.class, "Launcher servo");

        init_IMU();
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Front Distance Sensor");
        localizer = new AprilTagLocalizerTwo("Webcam 2", hardwareMap, telemetry, 0, 0);

        rightElbowServo = hardwareMap.get(Servo.class, "Right elbow servo");
        g2Y = new ButtonToggle();
        g2A = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        g1A = new ButtonToggle();
        g1RightBump = new ButtonToggle();
        g2X = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        g1LeftBump = new ButtonToggle();
        rightIntakeServo = hardwareMap.get(Servo.class, "Right intake servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");

        boxServo = hardwareMap.get(Servo.class, "Center box servo");
        boxServoController = new ServoSmooth(boxServo);

        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized");

        startingPositions [0] = motorLeft.getCurrentPosition();
        startingPositions [1] = motorRight.getCurrentPosition();
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

        if (g1LeftBump.update(gamepad1.left_bumper)) {
            isUsingFieldOriented = !isUsingFieldOriented;
            gamepad1.rumble(100);
        }

        if (!gamepad2.left_bumper) {
            if (!isUsingFieldOriented) {
                mecanumDrive.normalDrive(power, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }
            else {
                mecanumDrive.FieldOrientedDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x,
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2 + rotationResetConstant,
                        telemetry);
            }
        }

        if(gamepad1.dpad_up){
            alignToBoardContinuous();
        }

        if (gamepad1.a) { // Temporary for field oriented drive, may come up with auto align functionality
            imu.resetYaw();
            rotationResetConstant = Math.PI / 2; // Assumes resetting at 90˚ from starting position, aka facing backstage side
        }

        telemetry.addData("Driver mode", isUsingFieldOriented ? "Field Oriented" : "Normal");
        telemetry.addData("Driver speed", power == 1 ? "High" : "Low");
        telemetry.addData("Left side position", motorLeft.getCurrentPosition());
        telemetry.addData("Right side position", motorRight.getCurrentPosition());
    }

    @Override
    public void start() {
//        while (!boxServoController.setServoPosition(boxServoLeft, 1, telemetry)) {}
//        while (!boxServoController.setServoPosition(boxServoRight, 1, telemetry)) {}
//        while (!boxServoController.setServoPosition(boxServoRight, 1, telemetry)) {}

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

        if (gamepad2.left_bumper) {
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

    // For setting motor bounds and allowing automatic servo movement
    double [] mainExtensionConstants = {0.1, 0.1}; // For both sides to follow based on distance to target; left, right
    int [] slidesBounds = {-2629, 2974};
    int [] startingPositions = {0, 0};
    double inPerTickLeftSlide = -21.5 / 2629d;
    double inPerTickRightSlide = 21.5 / 2974d;

    void handleOuttakeControls() {
        double slidesDirection = gamepad2.a ? -1 : 1;
        motorLeft.setPower(slidesDirection * gamepad2.right_trigger);
        motorRight.setPower(slidesDirection * gamepad2.right_trigger);

        if (this.gamepad2.right_bumper) {
            if (gamepad2.a) {
                boxServo.setPosition(boxServoRight);
//                boxServoController.setServoPosition(boxServoNeutral, boxServoRight, 1, telemetry);  // right
            } else {
                boxServo.setPosition(boxServoLeft); //left
//                boxServoController.setServoPosition(boxServoNeutral, boxServoLeft, 0.5, telemetry);
//                boxServoController.setStartingPosition(boxServoNeutral);
            }
        } else {
            boxServo.setPosition(boxServoNeutral);  // center
        }

        if (g2X.update(gamepad2.x)) {
            if (wristPos == wristServoIn) { wristPos = wristServoOut; }
            else { wristPos = wristServoIn; }
            rightWristServo.setPosition(wristPos);
        }

        if (gamepad2.b) {
            rightWristServo.setPosition(wristServoFlat);
        }
    }

    void telemetry() {

    }

    void init_IMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    double backDropAligned = 3.5d; // CM
    double rotationCorrectionConstant = 0.05;
    public double alignToBoardContinuous(){
        if(rangeSensor.getDistance(DistanceUnit.CM) - backDropAligned > 0.5){
            mecanumDrive.normalDrive(1, 0.0, -(rangeSensor.getDistance(DistanceUnit.CM) - backDropAligned) * 0.05, rotationCorrectionConstant * (Math.PI - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.PI / 2d -  rotationResetConstant));
        }

        telemetry.addData("Distance to board", rangeSensor.getDistance(DistanceUnit.CM));

        return rangeSensor.getDistance(DistanceUnit.CM) - backDropAligned;
    }

    public boolean alignToLaunchPositionContinuous(){ // Uses april tags, has contingency to test whether or not  april tags are visible
        return false;
    }
}

