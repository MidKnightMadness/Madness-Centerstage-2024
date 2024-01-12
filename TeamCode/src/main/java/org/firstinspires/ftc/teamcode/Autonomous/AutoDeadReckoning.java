package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
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
public class AutoDeadReckoning extends OpMode implements WheelRPMConfig {
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }

    CameraModes cameraMode = getAllianceColor();
    public double getInchesToPark() {
        return 52;
    }

    DeadReckoningDrive deadReckoningDrive;
    AprilTagLocalizerTwo localizer;
    IMU imu;

    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;
    Servo intakeRightServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    Timer timer;
    ButtonToggle a, b, x, y;
    OpenCvWebcam webcam;
    public WebcamName webcamName;
    TeamPropMask teamPropMask;

    // April tag alignment
    double [] cameraCoordinates = {0.0, 0.0};
    int tagID = 0;
    double [][] targetCoordinates = {{118.5, 41d}, {118.5, 35d}, {118.5, 29d}};
    double xError = 0.0;
    double yError = 0.0;
    double lastXError = 0.0;
    double lastYError = 0.0;

    @Override
    public void init() {
        timer = new Timer();
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");

        teamPropMask = new TeamPropMask(640, 360, telemetry);
        teamPropMask.setMode(cameraMode);
        intakeRightServo = hardwareMap.get(Servo.class, "Right intake servo");
        intakeRightServo.setPosition(0.1);

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
    }

    @Override
    public void start() {
        webcam.stopStreaming();

        if(teamPropPosition == SpikeMarkPositions.LEFT){
            deadReckoningDrive.moveForwardDistance(18d);
            deadReckoningDrive.setTargetRotation(60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 8d);
            sleep(500);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 8d);
            deadReckoningDrive.setTargetRotation(-90);
        }else if(teamPropPosition == SpikeMarkPositions.RIGHT){
            deadReckoningDrive.moveForwardDistance(5d);
            deadReckoningDrive.setTargetRotation(-42);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 12d);
            sleep(500);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d);
            deadReckoningDrive.setTargetRotation(-90);
        }else{
            deadReckoningDrive.moveForwardDistance(27d);
            sleep(1000);
            deadReckoningDrive.moveForwardDistance(-9);
        }

        // go to backdrop
        if (teamPropPosition == SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveRightDistance(10d);
            deadReckoningDrive.moveForwardDistance(25d);
            deadReckoningDrive.moveRightDistance(-13);
            deadReckoningDrive.setTargetRotation(-90);
        } else {
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveForwardDistance(27);
        }

        // Aligning code, may be broken from here on
        if(teamPropPosition == SpikeMarkPositions.LEFT){
            deadReckoningDrive.moveRightDistance(-10d);
            deadReckoningDrive.setMotorPowersForTime(1.5d, 0.2, 0.2, 0.2, 0.2);
            double extensionStartingTime = timer.getTime();
            while(timer.getTime() < extensionStartingTime + 2){

            }
        }else if(teamPropPosition == SpikeMarkPositions.RIGHT){

        }else{

        }

        // Just testing tags detected
//        localizer = new AprilTagLocalizerTwo(webcamName, hardwareMap, telemetry, 0.0, 0.0);
//        init_IMU();
//        double startDetectionTime = timer.getTime();
//        while(timer.getTime() - startDetectionTime < 10000){
//            cameraCoordinates = localizer.getRelCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 0.0, 0.0);
//
//            telemetry.addData("imu yaw", getRobotDegrees());
//            if(cameraCoordinates != null){
//                telemetry.addData("Percieved x", cameraCoordinates [0]);
//                telemetry.addData("Percieved y", cameraCoordinates [1]);
//            }else{
//                telemetry.addLine("No detections");
//            }
//
//            telemetry.addData("P", -0.1);
//            telemetry.addData("D", -0.1);
//            telemetry.addLine("\n\n\n========================");
//        }


//        if(teamPropPosition == SpikeMarkPositions.LEFT){
//            tagID = 4;
//        }else if(teamPropPosition == SpikeMarkPositions.CENTER){
//            tagID = 5;
//        }else{
//            tagID = 6;
//        }
//        localizer = new AprilTagLocalizer(hardwareMap, telemetry, 0.0, 0.0);
//        init_IMU();
//        sleep(1000);
//        cameraCoordinates = localizer.getRelCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 0.0, 0.0);
//        xError = targetCoordinates [tagID - 4][0] - cameraCoordinates [0];
//        yError = targetCoordinates [tagID - 4][1] - cameraCoordinates [1];
//
//        while(Math.sqrt(xError*xError + yError * yError) > 0.25){ // Align to tag 2
//            cameraCoordinates = localizer.getRelCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 0.0, 0.0);
//
//            xError = targetCoordinates [tagID - 4][0] - cameraCoordinates [0];
//            yError = targetCoordinates [tagID - 4][1] - cameraCoordinates [1];
//
//            double dX = xError - lastXError;
//            double dY = yError - lastYError;
//
//            lastXError = xError;
//            lastYError = yError;
//
//            FieldOrientedDrive(-0.1 * (xError) + 0.1 * dX,
//                    -0.1 * (yError) + 0.1 * dY,
//                    0.0,
//                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), telemetry);
//
//            if(cameraCoordinates != null){
//                telemetry.addData("Percieved x", cameraCoordinates [0]);
//                telemetry.addData("Percieved y", cameraCoordinates [1]);
//            }else{
//                telemetry.addLine("No detections");
//            }
//            telemetry.update();
//        }
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
            deadReckoningDrive.moveRightDistance(24);
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

        telemetry.addData("KP", kp);

        deadReckoningDrive.setRotationKp(kp);
//
//        telemetry.clear();
//        telemetry.addData("Forward distance traveled", forwardDisplacement);
//        updateForwardDisplacement();
//
//        telemetry.addData("Left ticks", leftEncoder.getCurrentPosition());
//        telemetry.addData("Right ticks", rightEncoder.getCurrentPosition());
//
//        telemetry.addData("imu yaw", getRobotDegrees());
//        telemetry.addData("final error", finalerror);
//        telemetry.addData("KP", kP);
//
//        if (this.gamepad1.right_bumper) {
//            resetForwardDisplacement();
//        }

//        telemetryMotorVelocities();
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
}
