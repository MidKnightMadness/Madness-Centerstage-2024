package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp
public class AutoDeadReckoning extends OpMode implements WheelRPMConfig {

    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
    public DcMotorEx FL, FR, BL, BR, leftEncoder, rightEncoder;
    CameraModes cameraMode = getAllianceColor();
    public double getInchesToPark() {
        return 52;
    }
    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;
    Servo intakeRightServo;
    Timer timer;
    ButtonToggle a, b, x, y;
    OpenCvWebcam webcam;
    IMU imu;
    TeamPropMask teamPropMask;

    @Override
    public void init() {
        timer = new Timer();
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

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

        init_IMU();
        init_motors();
    }

    void init_IMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    void init_motors() {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
        resetEncoders();

        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

//        setMotorPowersForTime(2.5, -1, 1, -1, 1);
        park();
//
//
//        if (teamPropPosition == SpikeMarkPositions.LEFT || teamPropPosition == SpikeMarkPositions.RIGHT) {
//            int spikeMarkDirection = teamPropPosition == SpikeMarkPositions.LEFT ? 1 : -1;  // right: -1, left: 1
//            moveForwardDistance(24);
//            setTargetRotation(spikeMarkDirection * Math.PI / 2);
//            moveForwardDistance(5);
//            sleep(1000);
//            moveForwardDistance(-5);
//            setTargetRotation(0);
//            moveForwardDistance(-24);
//        }
//        else {
//            moveForwardDistance(33);
//            sleep(1000);
//            moveForwardDistance(-33);
//        }
//
//        // park
//        int parkingDirection = cameraMode == CameraModes.RED ? -1 : 1;  // red : turn right, blue : turn left
//        setTargetRotation(parkingDirection * Math.PI / 2);
//        moveForwardDistance(getInchesToPark());


    }

    void sleep(long milis) {
        try {
            Thread.sleep(milis);
        }
        catch (InterruptedException e) {
            telemetry.addData("Error", e.getMessage());
        }

    }

    void park() {
        setMotorPowersForTime(2.5, 1, 1, 1, 1);
    }


    @Override
    public void loop() {
        if (y.update(gamepad1.y)) {
            setTargetRotation(Math.PI / 2);
        }

        if (a.update(gamepad1.a)) {
            setTargetRotation(-Math.PI / 2);
        }

        if (b.update(gamepad1.b)) {
            moveForwardDistance(12);
        }
        if (x.update(gamepad1.x)) {
            setTargetRotation(0);
        }

        telemetry.clear();
        telemetry.addData("Forward distance traveled", forwardDisplacement);
        telemetry.addData("Left ticks", lastTicks[0]);
        telemetry.addData("Right ticks", lastTicks[1]);
        updateForwardDisplacement();

//        telemetryMotorVelocities();
    }

    final double IN_PER_TICK = 30.0d / 38888d;
    int[] lastTicks = new int[] { 0, 0 };
    double forwardDisplacement = 0;
    void updateForwardDisplacement() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();

        int deltaLeftTicks = leftPos - lastTicks[0];
        int deltaRightTicks = rightPos - lastTicks[1];

        lastTicks[0] = leftPos;
        lastTicks[1] = rightPos;

        // left encoder is reversed
//        forwardDisplacement += (-deltaLeftTicks + deltaRightTicks) * IN_PER_TICK / 2d;
        forwardDisplacement += -deltaLeftTicks * IN_PER_TICK;
    }

    void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        double maxPower = 0.5;
        double minPower = 0.1;

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
//            if (currentTime - startTime > 5) {
//                break;
//            }

            // power proportional to error between min and max power
            error = targetYawRadians - orientation.getYaw(AngleUnit.RADIANS);
            telemetry.clear();

            telemetry.update();

            double proportionalPower = (error / rotation) * (maxPower - minPower) + minPower;
            telemetry.addData("current rot", orientation.getYaw(AngleUnit.RADIANS)  * 180d / Math.PI);
            telemetry.addData("target rot", targetYawRadians * 180d / Math.PI);
            telemetry.addData("error", error * 180 / Math.PI);
            telemetry.addData("proportional power", proportionalPower);

            if (proportionalPower > maxPower) proportionalPower = maxPower;
            setPowers(-proportionalPower * direction, proportionalPower * direction,
                    -proportionalPower * direction, proportionalPower * direction);
            orientation = imu.getRobotYawPitchRollAngles();
            currentTime = timer.updateTime();
        }

        setPowers(0, 0, 0, 0);
    }

    @Deprecated
    void moveForwardDistanceByTime(double distance) {
        // dist = 28.57t - 10.02
        driveForwardForTime((distance + 10.02) / 28.57, 0.7);
    }

    void moveForwardDistance(double distance) {
        moveForwardDistanceByTime(distance);

//        double maxPower = 1;
//        double minPower = 0.1;
//        forwardDisplacement = 0;
//
//        double startTime = timer.updateTime();
//        double currentTime = startTime;
//
//        // -1 or 1
//        double direction = distance / Math.abs(distance);
//
//        while (forwardDisplacement < distance - 0.2) {
//            // hard cap
//            if (currentTime - startTime > 5) {
//                break;
//            }
//            updateForwardDisplacement();
//
//            double error = distance - forwardDisplacement;
//            double power = minPower + (maxPower - minPower) * (error / distance);
//            setMotorPowersSmoothed(power * direction, power * direction, power * direction, power * direction);
//        }
//
//        forwardDisplacement = 0;
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
