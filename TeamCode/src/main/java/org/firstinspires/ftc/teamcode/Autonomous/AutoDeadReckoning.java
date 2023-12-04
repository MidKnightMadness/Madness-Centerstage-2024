package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;
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
    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;

    Timer timer;

    ButtonToggle a;
    ButtonToggle b;
    ButtonToggle y;
    ButtonToggle x;

    OpenCvWebcam webcam;

    IMU imu;

    TeamPropMask teamPropMask;

    double [] RPMs = {390.0,
            186.8,
            389.2,
            186.7};

    double min = RPMs[3];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1] , min / RPMs[2], min / RPMs[3]};

    @Override
    public void init() {
        timer = new Timer();
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();



        teamPropMask = new TeamPropMask(640, 360, telemetry);
        teamPropMask.setMode(CameraModes.BLUE);

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

        init_IMU();
        init_motors();
    }

    void init_IMU() {
        RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
                = RevHubOrientationOnRobot.LogoFacingDirection.values();
        RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
                = RevHubOrientationOnRobot.UsbFacingDirection.values();

        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[0];  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[2];   // usb facing forward
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

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
        teamPropPosition = teamPropMask.getSpikeMarkPosition();
        telemetry.addData("imu yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("Detected spike mark position", teamPropPosition);
    }

    @Override
    public void start() {
        webcam.stopStreaming();
        moveForwardDistance(25);

        if (teamPropPosition == SpikeMarkPositions.LEFT) {
            setTargetRotation(Math.PI / 2);
            moveForwardDistance(10);
        }
        else if (teamPropPosition == SpikeMarkPositions.RIGHT) {
            setTargetRotation(-Math.PI / 2);
            moveForwardDistance(10);
        }
        else {
            moveForwardDistance(10);
        }

        moveForwardDistance(35.25);
    }

    @Override
    public void loop() {
        if (y.update(gamepad1.y)) {
            setTargetRotation(Math.PI / 2);
//            driveForwardForTime(3.5, 0.7);
        }

        if (a.update(gamepad1.a)) {
            setTargetRotation(-Math.PI / 2);
        }

        if (b.update(gamepad1.b)) {
            driveForwardForTime(1, 0.7);
        }
        if (x.update(gamepad1.x)) {
            setTargetRotation(0);
        }

//        telemetryMotorVelocities();
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

    double[] lastPowers = {0, 0, 0, 0};

    void setPowers(double fl, double fr, double bl, double br) {
        FL.setPower(fl * RPMMultipliers[0]);
        FR.setPower(fr * RPMMultipliers[1]);
        BL.setPower(bl * RPMMultipliers[2]);
        BR.setPower(br * RPMMultipliers[3]);

        lastPowers = new double[] {fl, fr, bl, br};
    }

    void setMotorPowersSmoothed(double flPow, double frPow, double blPow, double brPow) {
        double k = 1/50d;

        double fl = flPow * RPMMultipliers[0];
        double fr = frPow * RPMMultipliers[1];
        double bl = blPow * RPMMultipliers[2];
        double br = brPow * RPMMultipliers[3];

        FL.setPower(fl = lastPowers[0] * (1 - k) + fl * k);
        FR.setPower(fr = lastPowers[1] * (1 - k) + fr * k);
        BL.setPower(bl = lastPowers[2] * (1 - k) + bl * k);
        BR.setPower(br = lastPowers[3] * (1 - k) + br * k);

        lastPowers = new double[] {fl, fr, bl, br};
    }

    void driveForwardForTime(double seconds, double power) {
        setMotorPowersForTimeSmoothed(seconds, power, power, power, power);
    }

    void telemetryMotorVelocities() {
        telemetry.addData("FL RPM", radPSToRPM(FL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("FR RPM", radPSToRPM(FR.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BL RPM", radPSToRPM(BL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BR RPM", radPSToRPM(BR.getVelocity(AngleUnit.RADIANS)));
    }
    void telemetryMotorPowers() {
        telemetry.addData("FL Pow", FL.getPower());
        telemetry.addData("FR Pow", FR.getPower());
        telemetry.addData("BL Pow", BL.getPower());
        telemetry.addData("BR Pow", BR.getPower());
    }

    void setTargetRotation(double targetRotation) {
        double maxPower = 0.4;
        double minPower = 0.07;

        double percentToStop = 0.995;

        double startTime = timer.updateTime();
        double currentTime = startTime;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double startingYaw = orientation.getYaw(AngleUnit.RADIANS);

        double rotation = targetRotation - startingYaw;

        double targetYawRadians = startingYaw + rotation;

        double error = rotation;
        if (rotation - startingYaw < 0.3) {
            maxPower = 0.2;
        }

        double direction = rotation / Math.abs(rotation);

        while (Math.abs(error) > Math.abs((1 - percentToStop) * rotation)) {
            // hard cap
            if (currentTime - startTime > 5) {
                break;
            }

            // power proportional to error between min and max power
            error = targetYawRadians - orientation.getYaw(AngleUnit.RADIANS);
            telemetry.clear();
            telemetry.addData("current rot", orientation.getYaw(AngleUnit.RADIANS)  * 180d / Math.PI);
            telemetry.addData("target rot", targetYawRadians * 180d / Math.PI);
            telemetry.addData("error", error * 180 / Math.PI);
            telemetry.update();

            double proportionalPower = (error / rotation) * (maxPower - minPower) + minPower;

            if (proportionalPower > maxPower) proportionalPower = maxPower;
            setPowers(-proportionalPower * direction, proportionalPower * direction,
                    -proportionalPower * direction, proportionalPower * direction);
            orientation = imu.getRobotYawPitchRollAngles();
            currentTime = timer.updateTime();

        };

        setPowers(0, 0, 0, 0);

    }

    void moveForwardDistance(double distance) {
        // dist = 28.57t - 10.02
        driveForwardForTime((distance + 10.02) / 28.57, 0.7);
    }


    void setMotorPowersForTimeSmoothed(double seconds, double fl, double fr, double bl, double br) {
        double startTime = timer.updateTime();

        // run for time
        while (timer.getTime() - startTime < seconds) {
            setMotorPowersSmoothed(fl, fr, bl, br);
//            telemetryMotorVelocities();
            timer.updateTime();
        }

        while (Math.abs(FL.getPower()) > 0.1) {
            setMotorPowersSmoothed(0, 0, 0, 0);
        }

        setPowers(0, 0, 0, 0);
    }

    void setMotorPowersForTime(double seconds, double fl, double fr, double bl, double br) {
        double startTime = timer.updateTime();

        // run for time
        while (timer.getTime() - startTime < seconds) {
            setPowers(fl, fr, bl, br);
//            telemetryMotorVelocities();
            timer.updateTime();
        }

        setPowers(0, 0, 0, 0);
    }

}
