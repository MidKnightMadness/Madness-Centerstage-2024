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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
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

    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;
    Servo intakeRightServo;
    Timer timer;
    ButtonToggle a, b, x, y;
    OpenCvWebcam webcam;
    TeamPropMask teamPropMask;

    @Override
    public void init() {
        timer = new Timer();
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);

        teamPropMask = new TeamPropMask(640, 360, telemetry);
        teamPropMask.setMode(cameraMode);
        intakeRightServo = hardwareMap.get(Servo.class, "Right intake servo");
        intakeRightServo.setPosition(0.1);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
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

        if (teamPropPosition == SpikeMarkPositions.LEFT || teamPropPosition == SpikeMarkPositions.RIGHT) {
            int spikeMarkDirection = teamPropPosition == SpikeMarkPositions.LEFT ? 1 : -1;  // right: -1, left: 1
            deadReckoningDrive.moveForwardDistance(24);
            deadReckoningDrive.setTargetRotation(spikeMarkDirection * 90);
            deadReckoningDrive.moveForwardDistance(5);
            sleep(1000);
            deadReckoningDrive.moveForwardDistance(-5);
            deadReckoningDrive.setTargetRotation(-90);
        }
        else {
            deadReckoningDrive.moveForwardDistance(33);
            sleep(1000);
            deadReckoningDrive.moveForwardDistance(-9);
        }

        // go to backdrop
        if (teamPropPosition == SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveRightDistance(16);
            deadReckoningDrive.moveForwardDistance(30);
            deadReckoningDrive.moveRightDistance(-18);
            deadReckoningDrive.setTargetRotation(-90);
        } else {
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveForwardDistance(30);
        }
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

}
