package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Components.LinearSlides;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/* For testing opmodes without odometry
Objectives:
1. Driving forward by distance, whether by wheel radii calculations or by calibrated time and power calculations
2. Smooithly ramp up and ramp down power
Contains:
1. Drive using power and time (primarily used)
2. Drive using distance
3. Conversion functions and compensation for RPM-torque differences between motors
 */
@TeleOp
@SuppressLint("DefaultLocale")
public class AutoDeadReckoning extends OpMode implements WheelRPMConfig, ServoPositions {
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
    Thread thread;
    public StartingPosition getStartingPosition() {
        return StartingPosition.NEAR;
    }
    public double slidesExtensionTimeConstant = 1.9;
    public double rammingPower = 0.6;

    CameraModes cameraMode = getAllianceColor();
    public DeadReckoningDrive deadReckoningDrive;
    IMU imu;
    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;
    Servo intakeRightServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Timer timer;
    ButtonToggle a, b, x, y;
    OpenCvWebcam webcam;
    public WebcamName webcamName;
    TeamPropMask teamPropMask;
    LinearSlides slides;
    ServoSmooth boxServoController;

    @Override
    public void init() {
        timer = new Timer();
        thread = new Thread();
        slides = new LinearSlides(hardwareMap);
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");
        boxServo = hardwareMap.get(Servo.class, "Center box servo");
        boxServoController = new ServoSmooth(boxServo);
        init_IMU();

        teamPropMask = new TeamPropMask(640, 360, telemetry);
        teamPropMask.setMode(getAllianceColor());
        intakeRightServo = hardwareMap.get(Servo.class, "Right intake servo");
        intakeRightServo.setPosition(0.1);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Front Distance Sensor");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(teamPropMask);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error " + errorCode, "error accessing camera stream");
            }
        });
    }

    @Override
    public void init_loop() {
        telemetry.clear();
        teamPropPosition = teamPropMask.getSpikeMarkPosition();
        telemetry.addData("imu yaw", deadReckoningDrive.getRobotDegrees());
        telemetry.addData("Detected spike mark position", teamPropPosition);
        deadReckoningDrive.updateDisplacement();
        telemetry.addData("Displacement", deadReckoningDrive.getDisplacement());
        if(cameraMode == CameraModes.RED){
            telemetry.addLine("Camera Mode: RED");
        }else{
            telemetry.addLine("Camera Mode: BLUE");
        }
    }

    @Override
    public void start() {
        imu.resetYaw();
        webcam.stopStreaming();

        // Reset servos
        boxServo.setPosition(boxServoNeutral);
        rightWristServo.setPosition(wristServoIn);
        intakeRightServo.setPosition(intakeHighest);

        drive();
    }

    @Override
    public void stop(){
        telemetry.addLine("STOPPED");
        telemetry.update();
    }

    void sleep(long milis) {
        try {
            Thread.sleep(milis);
        }
        catch (InterruptedException e) {
            telemetry.addData("Error", e.getMessage());
        }

    }
    double increment = 0.0005;
    double kp = 0.2;

    @Override
    public void loop() {
        telemetry.clear();

        if (y.update(gamepad1.y)) {
//            deadReckoningDrive.moveRightDistance(-24);
            deadReckoningDrive.setTargetRotation(0);
//            setTargetRotation(Math.round(getRobotDegrees()) + 90);
//            driveForwardForTime(2, 0.5);
        }
//
        if (a.update(gamepad1.a)) {
            deadReckoningDrive.setTargetRotation(180);
//            setTargetRotation(Math.round(getRobotDegrees()) - 90);
//            driveForward(12);
        }
//
        if (b.update(gamepad1.b)) {
            deadReckoningDrive.setTargetRotation(90);
        }
        if (x.update(gamepad1.x)) {
            deadReckoningDrive.setTargetRotation(-90);
        }
//
//        if (gamepad1.left_bumper) {
//            setPowers(1, 1, 1, 1);
//        }
//        else {
//            setPowers(0, 0, 0, 0);
//        }
//
        if (this.gamepad1.dpad_up) {
            kp += increment;
        }

        if (gamepad1.dpad_down) {
            kp -= increment;
        }

        telemetry.addData("Robot degrees", deadReckoningDrive.getRobotDegrees());
    }

    void init_IMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    double [] motorInputs = {0.0, 0.0, 0.0, 0.0};
    double previousX = 0.0;
    double previousY = 0.0;
    public static final double [] FORWARD = {-1.0, 1.0, -1.0, -1.0};
    ///forward: -1.0, 1.0, -1.0, -1.0
    public static final double [] RIGHT = {-1.0, -1.0, 1.0, -1.0};
    public static final double [] CLOCKWISE = {-1.0, -1.0, -1.0, 1.0};
    public static final double POWER_MULTIPLIER = 1;
    public void FieldOrientedDrive(double x, double y, double rotation, double angle, Telemetry telemetry){ // Angle of front from horizontal right, meant for controller inputs
        double maxPowerLevel = 0.0;

        // Low pass
        double lowPassX = 0.05 * x + 0.95 * previousX;
        double lowPassY = 0.05 * y + 0.95 * previousY;


        // Rotate x and y by negative of angle
        double newX = lowPassX*Math.cos(angle - (Math.PI / 2.0)) + lowPassY*Math.sin(angle - (Math.PI / 2.0));
        double newY = -lowPassX*Math.sin(angle - (Math.PI / 2.0)) + lowPassY*Math.cos(angle - (Math.PI / 2.0));

        // Update low pass previous variables
        previousX = x;
        previousY = y;

        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * newY) + (RIGHT [i] * newX) + (CLOCKWISE [i] * rotation)) * POWER_MULTIPLIER * RPMMultipliers[i] ;

            if(Math.abs(motorInputs [i]) > maxPowerLevel){
                maxPowerLevel = Math.abs(motorInputs [i]);
            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]);
        if(powerEnvelope > 0.2 && maxPowerLevel > 1.0){
            for(int i = 0; i < 4; i++){
                motorInputs [i] /= maxPowerLevel;
            }
        }

//        telemetry.addData("\nPower envelope", powerEnvelope);
//        telemetry.addData("Max Power", maxPowerLevel);
//        telemetry.addData("X", newX);
//        telemetry.addData("Y", newY);
//
        telemetry.addData("FL", motorInputs [0]);
        telemetry.addData("FR", motorInputs [1]);
        telemetry.addData("BL", motorInputs [2]);
        telemetry.addData("BR", motorInputs [3]);
//        telemetry.addData("Low pass latency", 0.5);

        setMotorPowers();
    }

    void setMotorPowers() {
        deadReckoningDrive.FL.setPower( motorInputs [0]);
        deadReckoningDrive.FR.setPower( motorInputs [1]);
        deadReckoningDrive.BL.setPower( motorInputs [2]);
        deadReckoningDrive.BR.setPower( motorInputs [3]);
    }

    double getRobotDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public void drive(){}

    public void rotateBoxTo(double position) {
        double servoPosition = boxServo.getPosition();
        while(boxServo.getPosition() < position){
            boxServo.setPosition(boxServo.getPosition() + 0.01);
            try {
                Thread.sleep((long) Math.round(10.0 * Math.PI / (Math.PI)));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
