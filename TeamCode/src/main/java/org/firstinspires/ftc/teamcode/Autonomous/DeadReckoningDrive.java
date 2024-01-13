package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

public class DeadReckoningDrive implements WheelRPMConfig {
    Vector2 position = new Vector2();

    IMU imu;
    Timer timer;
    Telemetry telemetry;

    double rotationKp = 0.2;

    public void setRotationKp(double kp) {
        this.rotationKp = kp;
    }

    double movementKp = 0.2;

    public DcMotorEx FL, FR, BL, BR, leftEncoder, rightEncoder, topEncoder;

    public DeadReckoningDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        init_IMU(hardwareMap);
        init_motors(hardwareMap);

        this.telemetry = telemetry;
        timer = new Timer();
    }

    void init_IMU(HardwareMap hardwareMap) {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    void init_motors(HardwareMap hardwareMap) {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
        topEncoder = hardwareMap.get(DcMotorEx.class, "Intake motor");

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

    final double TICKS_PER_INCH = 1248.66631083;
    double forwardDisplacement = 0;
    double lateralDisplacement = 0;

    double kp = 0;

    void updateDisplacement() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();

        // left encoder is reversed
//        forwardDisplacement += (-deltaLeftTicks + deltaRightTicks) * IN_PER_TICK / 2d;
        forwardDisplacement = (-leftPos+ rightPos) / (2d * TICKS_PER_INCH);
        lateralDisplacement = topEncoder.getCurrentPosition() / TICKS_PER_INCH;
    }
    void resetDisplacement() {
        resetEncoders();
        forwardDisplacement = 0;
        lateralDisplacement = 0;
    }

    void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Vector2 getDisplacement() {
        return new Vector2(lateralDisplacement, forwardDisplacement);
    }

    void updatePosition(double rotation, double distance) {
        double updatedRotation = rotation / 180 * Math.PI + Math.PI / 2;

        position.x += Math.cos(updatedRotation) * distance;
        position.y += Math.sin(updatedRotation) * distance;
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

    void telemetryMotorVelocities(Telemetry telemetry) {
        telemetry.addData("FL RPM", radPSToRPM(FL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("FR RPM", radPSToRPM(FR.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BL RPM", radPSToRPM(BL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BR RPM", radPSToRPM(BR.getVelocity(AngleUnit.RADIANS)));
    }
    void telemetryMotorPowers(Telemetry telemetry) {
        telemetry.addData("FL Pow", FL.getPower());
        telemetry.addData("FR Pow", FR.getPower());
        telemetry.addData("BL Pow", BL.getPower());
        telemetry.addData("BR Pow", BR.getPower());
    }


    void setTargetRotation(double targetRotation) {
        targetRotation = normalizeAngle(targetRotation);

        double maxPower = 0.7;
        double minPower = 0.225;

        double startingYaw = getRobotDegrees();

        double rotation = normalizeAngle(targetRotation - startingYaw);

        double error = rotation;
//
        double startTime = timer.updateTime();

        if (rotation - startingYaw < 1) {
            maxPower = minPower;
        }

        double degreesTillStop = 0.18;
        // stops if within degrees
        while (Math.abs(error) > degreesTillStop) {
            if (startTime - timer.updateTime() > 10)  degreesTillStop += 0.1;

            // power proportional to error between min and max power
            error = normalizeAngle(targetRotation - getRobotDegrees());

            double proportionalPower = Math.abs(error / 90d) * (maxPower - minPower) + minPower;
            double direction = Math.signum(error);

            proportionalPower = Math.min(maxPower, proportionalPower);

            telemetry.clear();
            telemetry.addData("error", error);
            telemetry.addData("rotation", rotation);

            telemetry.addData("kp", rotationKp);
            telemetry.addData("proportional power", proportionalPower * direction);
            telemetry.update();

            setPowers(-proportionalPower * direction, proportionalPower * direction,
                    -proportionalPower * direction, proportionalPower * direction);
        }

        setPowers(0, 0, 0, 0);
    }

    double clamp(double min, double max, double val) {
        if (val < min) return min;
        else if (val > max) return max;

        return val;
    }

    double clamp01(double val) {
        return clamp(0, 1, val);
    }

    double getRobotDegrees() {
        return normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    double normalizeAngle(double angle) {
        return mod((angle + 180), 360) - 180;
    }

    double mod(double num, double divisor) {
        return num - Math.floor(num / divisor) * divisor;
    }

    @Deprecated
    void moveForwardDistanceByTime(double distance) {
        // dist = 28.57t - 10.02
        driveForwardForTime((distance + 10.02) / 28.57, 0.7);
    }

    double kP = 1/16d;
    void moveForwardDistance(double distance) {
        double minPower = 0.225;
        double maxPower = 0.5;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;

        while (Math.abs(error) > 0.1) {
            if (currentTime - startTime > 5) {
                break;
            }
            updateDisplacement();

            error = distance - forwardDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / 16d);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * direction, power * direction, power * direction, power * direction);
        }

        setPowers(0, 0, 0, 0);
    }

    void moveRightDistance(double distance) {
        double minPower = 0.3;
        double maxPower = 0.5;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;

        while (Math.abs(error) > 0.1) {
            if (currentTime - startTime > 5) {
                break;
            }
            updateDisplacement();

            error = distance - lateralDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / distance);

            telemetry.clear();
            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * direction, power * -direction, power * -direction, power * direction);
        }

        setPowers(0, 0, 0, 0);
    }

    @Deprecated
    void driveForward(double distance) {
        double minPower = 0.225;
        double maxPower = 0.5;

        resetDisplacement();
        double error;

        do {
            updateDisplacement();

            error = distance - forwardDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error) * kP;

            power = Math.min(power, maxPower) * direction;

            setPowers(power, power, power, power);

            telemetry.clear();
            telemetry.addLine(String.format("Power: %.3f", power));
            telemetry.addLine(String.format("Error: %.2f", error));
            telemetry.addLine(String.format("Forward displacement %.3f", forwardDisplacement));
            telemetry.update();

        } while (Math.abs(error) > 0.1);

        setPowers(0, 0, 0, 0);
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