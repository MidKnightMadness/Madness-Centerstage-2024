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
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
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

    // Variables for aligning to april tags
    AprilTagLocalizer localizer;
    double [] cameraCoordinates = {0.0, 0.0};
    double [] targetCoordinates = {118.5, 35d};
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

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        telemetry.addData("imu yaw", getRobotDegrees());
        telemetry.addData("Detected spike mark position", teamPropPosition);
    }

    @Override
    public void start() {
        webcam.stopStreaming();

        if (teamPropPosition == SpikeMarkPositions.LEFT || teamPropPosition == SpikeMarkPositions.RIGHT) {
            int spikeMarkDirection = teamPropPosition == SpikeMarkPositions.LEFT ? 1 : -1;  // right: -1, left: 1
            moveForwardDistance(24);
            setTargetRotation(spikeMarkDirection * 90);
            moveForwardDistance(5);
            sleep(1000);
            moveForwardDistance(-5);
            setTargetRotation(0);
//            moveForwardDistance(-24);
        }
        else {
            moveForwardDistance(33);
            sleep(1000);
            moveForwardDistance(-33);
        }

        // park
//        int parkingDirection = cameraMode == CameraModes.RED ? -1 : 1;  // red : turn right, blue : turn left
//        setTargetRotation(parkingDirection * 90);
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

    double increment = 0.0005;


    @Override
    public void loop() {
        if (y.update(gamepad1.y)) {
            setTargetRotation(getRobotDegrees() + 90);
//            driveForwardForTime(2, 0.5);
        }

        if (a.update(gamepad1.a)) {
            setTargetRotation(getRobotDegrees() - 90);
//            driveForward(12);
        }

        if (b.update(gamepad1.b)) {
            moveForwardDistance(12);
        }
        if (x.update(gamepad1.x)) {
            setTargetRotation(0);
        }

        if (gamepad1.left_bumper) {
            setPowers(1, 1, 1, 1);
        }
        else {
            setPowers(0, 0, 0, 0);
        }

        if (this.gamepad1.dpad_up) {
            kP += increment;
        }
        if (gamepad1.dpad_down) {
            kP -= increment;
        }

        telemetry.clear();
        telemetry.addData("Forward distance traveled", forwardDisplacement);
        updateForwardDisplacement();

        telemetry.addData("Left ticks", leftEncoder.getCurrentPosition());
        telemetry.addData("Right ticks", rightEncoder.getCurrentPosition());

        telemetry.addData("imu yaw", getRobotDegrees());
        telemetry.addData("final error", finalerror);
        telemetry.addData("KP", kP);

        if (this.gamepad1.right_bumper) {
            resetForwardDisplacement();
        }

//        telemetryMotorVelocities();
    }

//    final double IN_PER_TICK = 0.00079829719;
    final double TICKS_PER_INCH = 1248.66631083;
    double forwardDisplacement = 0;
    void updateForwardDisplacement() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();

        // left encoder is reversed
//        forwardDisplacement += (-deltaLeftTicks + deltaRightTicks) * IN_PER_TICK / 2d;
        forwardDisplacement = (-leftPos+ rightPos) / (2d * TICKS_PER_INCH);
    }

    void resetForwardDisplacement() {
        resetEncoders();
        forwardDisplacement = 0;
    }
    void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    void setPowersSmoothed(double flPow, double frPow, double blPow, double brPow) {
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
        setPowersForTimeSmoothed(seconds, power, power, power, power);
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

    double finalerror = 0;

    void setTargetRotation(double targetRotation) {
        targetRotation = normalizeAngle(targetRotation);

        double maxPower = 0.7;
        double minPower = 0.225;

        double percentToStop = 0.9995;

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double startingYaw = getRobotDegrees();

        double rotation = normalizeAngle(targetRotation - startingYaw);
        double targetYawDegrees = startingYaw + rotation;

        double error = rotation;

        if (rotation - startingYaw < 1) {
            maxPower = minPower;
        }

        // -1 for right, 1 for left


        // stops if within 0.25 degrees
        while (Math.abs(error) > 0.18) {
            // hard cap
//            if (currentTime - startTime > 5) {
//                break;
//            }

            // power proportional to error between min and max power
            error = normalizeAngle( normalizeAngle(targetYawDegrees) - getRobotDegrees());
            telemetry.clear();

            double proportionalPower = Math.abs((error / rotation)) * (maxPower - minPower) + minPower;
            double direction = Math.signum(error);

            telemetry.addData("current rot", getRobotDegrees());
            telemetry.addData("target rot", targetYawDegrees);
            telemetry.addData("error", error);
            telemetry.addData("proportional power", proportionalPower * direction);
            telemetry.update();

            if (proportionalPower > maxPower) proportionalPower = maxPower;
            setPowers(-proportionalPower * direction, proportionalPower * direction,
                    -proportionalPower * direction, proportionalPower * direction);

            currentTime = timer.updateTime();
        }

        finalerror = error;

        setPowers(0, 0, 0, 0);
    }

    double getRobotDegrees() {
        return normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    double normalizeAngle(double angle) {
        return (angle + 180) % 360 - 180;
    }

    @Deprecated
    void moveForwardDistanceByTime(double distance) {
        // dist = 28.57t - 10.02
        driveForwardForTime((distance + 10.02) / 28.57, 0.7);
    }

    double kP = 1/12d;
    void moveForwardDistance(double distance) {
//        moveForwardDistanceByTime(distance);
        double minPower = 0.225;
        double maxPower = 0.5;

        resetForwardDisplacement();

//        double startTime = timer.updateTime();
//        double currentTime = startTime;

        double error = distance;
        double direction = Math.signum(distance);

        while (Math.abs(error) > 0.1) {
            // hard cap
//            if (currentTime - startTime > 5) {
//                break;
//            }
            updateForwardDisplacement();

            error = distance - forwardDisplacement;

            double power = minPower + (maxPower - minPower) * Math.abs(error / distance);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetryMotorPowers();
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * direction, power * direction, power * direction, power * direction);
        }

        setPowers(0, 0, 0, 0);
        finalerror = error;
    }

    void driveForward(double distance) {
        double minPower = 0.225;
        double maxPower = 0.5;

        resetForwardDisplacement();
        double error;

        do {
            updateForwardDisplacement();

            error = distance - forwardDisplacement;
            double power = minPower + (maxPower - minPower) * Math.abs(error) * kP;
            double direction = Math.signum(error);

            power = Math.min(power, maxPower) * direction;

            setPowers(power, power, power, power);

            telemetry.clear();
            telemetry.addLine(String.format("Power: %.3f", power));
            telemetry.addLine(String.format("Error: %.2f", error));
            telemetry.addLine(String.format("Forward displacement %.3f", forwardDisplacement));
            telemetry.update();

        } while (Math.abs(error) > 0.1);

        setPowers(0, 0, 0, 0);

        finalerror = error;

    }
    void setPowersForTimeSmoothed(double seconds, double fl, double fr, double bl, double br) {
        double startTime = timer.updateTime();

        // run for time
        while (timer.getTime() - startTime < seconds) {
            setPowersSmoothed(fl, fr, bl, br);
//            telemetryMotorVelocities();
            timer.updateTime();
        }

        while (Math.abs(FL.getPower()) > 0.1) {
            setPowersSmoothed(0, 0, 0, 0);
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
