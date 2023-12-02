package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp
public class AutoDeadReckoning extends OpMode {
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;

    CameraModes mode = CameraModes.RED;
    SpikeMarkPositions position = SpikeMarkPositions.LEFT;



    Timer timer;

    ButtonToggle a;
    ButtonToggle b;
    ButtonToggle y;
    ButtonToggle x;

    OpenCvWebcam webcam;
    TeamPropMask teamPropMask;

    double [] RPMs = {248.7,
            186.5,
            249.1 ,
            186.7};

    double min = RPMs[1];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1] , min / RPMs[2], min / RPMs[3]};

    @Override
    public void init() {
        timer = new Timer();

        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

        teamPropMask = new TeamPropMask(640, 360, telemetry);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
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


        init_motors();


    }

    void init_motors() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init_loop() {
        position = teamPropMask.getSpikeMarkPosition();
        telemetry.addData("Detected spike mark position", position);
    }

    double seconds = 1.3;

    @Override
    public void start() {
        webcam.stopStreaming();

        moveForwardDistance(35.25);
    }


    @Override
    public void loop() {
        if (y.update(gamepad1.y)) {
            rotate90Degrees();
        }

        if (a.update(gamepad1.a)) {
            driveForwardForTime(0.5, 0.7);
        }

        if (b.update(gamepad1.b)) {
            driveForwardForTime(1, 0.7);
        }
        if (x.update(gamepad1.x)) {
            driveForwardForTime(1.5, 0.7);
        }

        telemetry.addData("Rotation time", seconds);

        telemetryMotorVelocities();
    }

    double radPSToRPM(double radiansPerSec) {
        return radiansPerSec * 30d / Math.PI;
    }

    double RPMtoRadPS(double rpm) {
        return rpm / 30d * Math.PI;
    }

    void setMotorVelocities(double flRPM, double frRPM, double blRPM, double brRPM) {
        FL.setVelocity(RPMtoRadPS(flRPM), AngleUnit.RADIANS);
        FR.setVelocity(RPMtoRadPS(frRPM) * 25d/16, AngleUnit.RADIANS);
        BL.setVelocity(RPMtoRadPS(blRPM), AngleUnit.RADIANS);
        BR.setVelocity(RPMtoRadPS(brRPM) * 25d/16, AngleUnit.RADIANS);
    }

    void setMotorPowers(double flPow, double frPow, double blPow, double brPow) {
        FL.setPower(flPow * 16d/25 * RPMMultipliers[0] * 1.0225d);
        FR.setPower(frPow * RPMMultipliers[1]);
        BL.setPower(blPow * 16d/25 * RPMMultipliers[2] * 1.0225d);
        BR.setPower(brPow * RPMMultipliers[3]);
    }

    void driveForwardForTime(double seconds, double power) {
        setMotorPowersForTime(seconds, power, power, power, power);
    }

    void telemetryMotorVelocities() {
        telemetry.addData("FL RPM", radPSToRPM(FL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("FR RPM", radPSToRPM(FR.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BL RPM", radPSToRPM(BL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BR RPM", radPSToRPM(BR.getVelocity(AngleUnit.RADIANS)));
    }

    void rotate90Degrees() {
        double power = 0.5;
        double seconds = 1.29 * 1.65;
        setMotorPowersForTime(seconds, power, -power, power, -power);
    }

    void moveForwardDistance(double distance) {
        // dist = 25.65t - 3.83
        driveForwardForTime((distance + 3.83) / 25.65, 0.7);
    }


    void setMotorPowersForTime(double seconds, double fl, double fr, double bl, double br) {
        final double smoothInTime = 0.25;
        final double smoothOutTime = 0.25;
//        seconds -= (smoothInTime / smoothInTime) / 2d;

        double[] motorPowers = {fl, fr, bl, br};

        timer.updateTime();
        double startTime = timer.getTime();


        // smooth in
        double timeSinceStart = 0;
        while ((timeSinceStart = timer.getTime() - startTime) < smoothInTime) {
            double multiplier = timeSinceStart / smoothInTime;
            setMotorPowers(fl * multiplier, fr * multiplier, bl * multiplier, br * multiplier);
            timer.updateTime();
        }

        // run for time
        while (timer.getTime() - startTime < seconds) {
            setMotorPowers(fl, fr, bl, br);
            telemetryMotorVelocities();
            timer.updateTime();
        }

        startTime = timer.updateTime();

        // smooth out
        timeSinceStart = 0;
        while ((timeSinceStart = timer.getTime() - startTime) < smoothOutTime) {
            double multiplier = 1 - (timeSinceStart / smoothOutTime);
            setMotorPowers(fl * multiplier, fr * multiplier, bl * multiplier, br * multiplier);
            timer.updateTime();
        }

        setMotorPowers(0, 0, 0, 0);
    }

}
