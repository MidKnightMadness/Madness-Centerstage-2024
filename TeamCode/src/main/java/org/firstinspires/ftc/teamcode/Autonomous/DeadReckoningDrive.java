package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        topEncoder = hardwareMap.get(DcMotorEx.class, "BR");

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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


    // + -> -    (-)
    // - -> +    (-)
    //
    void setPowersSmoothed(double flPow, double frPow, double blPow, double brPow, double k) {
        double fl = flPow * RPMMultipliers[0];
        double fr = frPow * RPMMultipliers[1];
        double bl = blPow * RPMMultipliers[2];
        double br = brPow * RPMMultipliers[3];

        FL.setPower(fl = Math.abs(lastPowers[0]) * Math.signum(fl) * (1 - k) + fl * k);
        FR.setPower(fr = Math.abs(lastPowers[1]) * Math.signum(fr) * (1 - k) + fr * k);
        BL.setPower(bl = Math.abs(lastPowers[2]) * Math.signum(bl) * (1 - k) + bl * k);
        BR.setPower(br = Math.abs(lastPowers[3]) * Math.signum(br) * (1 - k) + br * k);

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
    int lastErrorCheckLength = 8;
    double lastErrorCheck;

    void setTargetRotation(double targetRotation, double maxPower) {
        targetRotation = normalizeAngle(targetRotation);

        double minPower = 0.175;

        double startingYaw = getRobotDegrees();
        double rotation = normalizeAngle(targetRotation - startingYaw);

        double error = rotation;
        double startTime = timer.updateTime();

        double degreesTillStop = 0.1;
        while (Math.abs(error) > degreesTillStop) {

            if (startTime - timer.updateTime() > 10)  degreesTillStop += 0.1;

            // power proportional to error between min and max power
            error = normalizeAngle(targetRotation - getRobotDegrees());

            double percentOfMaxPower = Math.min(1d, Math.abs(error / 120d));
            double proportionalPower = percentOfMaxPower * (maxPower - minPower) + minPower;
            double direction = Math.signum(error);

            telemetry.clear();
            telemetry.addData("robot angle", getRobotDegrees());
            telemetry.addData("error", error);
            telemetry.addData("rotation", rotation);

            telemetry.addData("proportional power", proportionalPower);
            telemetry.addData("direction", direction);

            telemetry.update();

//            setPowersSmoothed(proportionalPower * -direction, proportionalPower * direction,
//                    proportionalPower * -direction, proportionalPower * direction, 1/12d);
            setPowers(proportionalPower * -direction, proportionalPower * direction,
                    proportionalPower * -direction, proportionalPower * direction);
        }

        setPowers(0, 0, 0, 0);
    }

    void setTargetRotation(double targetRotation) {
        setTargetRotation(targetRotation, 0.5);
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
    public void moveForwardDistanceByTime(double distance) {
        // dist = 28.57t - 10.02
        driveForwardForTime((distance + 10.02) / 28.57, 0.7);
    }

    public void moveForwardDistance(double distance, double maxPower) {
        double minPower = 0.15;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
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

    public void moveForwardDistance(double distance, double maxPower, double seconds, boolean timed) {
        double minPower = 0.15;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop && timer.updateTime() - startTime < seconds) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
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
    public void moveForwardDistance(double distance) {
        moveForwardDistance(distance, 0.5);
    }

    public void moveForwardDistance(double distance, double maxPower, double targetAngle) {
        double currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
        double minPower = 0.16;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        int updates = 0;
        double rotationCorrection = 0;
        while (Math.abs(error) > errorToStop) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            // Makes sure angle is between 0 and 360˚
            if(updates++ % 10 == 0) {
                currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0) ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
                rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                if (rotationCorrection < 0.5) {
                    rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                } else {
                    rotationCorrection = 0;
                }
            }

            error = distance - forwardDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / 16d);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection), power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection));
        }

        setPowers(0, 0, 0, 0);
    }

    public void moveForwardDistance(double distance, double maxPower, double targetAngle, double seconds) {
        double currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
        double minPower = 0.16;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        int updates = 0;
        double rotationCorrection = 0;
        while (Math.abs(error) > errorToStop && timer.updateTime() - startTime < seconds) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            // Makes sure angle is between 0 and 360˚
            if(updates++ % 10 == 0) {
                currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0) ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
                rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                if (rotationCorrection < 0.5) {
                    rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                } else {
                    rotationCorrection = 0;
                }
            }

            error = distance - forwardDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / 16d);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection), power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection));
        }

        setPowers(0, 0, 0, 0);
    }



    public void moveRightDistance(double distance) {
        double minPower = 0.3;
        double maxPower = 0.5;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
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

    private final double rotationCorrectionConstant = 0.0;
    private final double rotationCorrectionConstantForRotation = 0.15;
    public void moveRightDistance(double distance, double targetAngle) { // Angle in radians
        double currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
        double minPower = 0.3;
        double maxPower = 0.5;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            // Makes sure angle is between 0 and 360˚
            currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
            double rotationCorrection = targetAngle * Math.PI / 180 - currentAngleCorrected;

            error = distance - lateralDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / distance);

            telemetry.clear();
            telemetry.addData("Error", error);
            telemetry.addData("Angle error", rotationCorrection);
            telemetry.addData("Rotation Correction Power", rotationCorrectionConstantForRotation * rotationCorrection);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * (direction + rotationCorrectionConstantForRotation * rotationCorrection), power * -(direction - rotationCorrectionConstantForRotation * rotationCorrection), power * -(direction + rotationCorrectionConstantForRotation * rotationCorrection), power * (direction - rotationCorrectionConstantForRotation * rotationCorrection));
        }

        setPowers(0, 0, 0, 0);
    }

    double backDropToWallTolerance = 10d;
    void strafeUntilBackdrop(ModernRoboticsI2cRangeSensor rangeSensor, boolean right) {
        double minPower = 0.3;
        double maxPower = 0.5;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;
        double initialDistance = rangeSensor.getDistance(DistanceUnit.CM);

        double errorToStop = 0.1;
        while (rangeSensor.getDistance(DistanceUnit.CM) < initialDistance - backDropToWallTolerance) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            double power = minPower + (maxPower - minPower) * 0.25;

            telemetry.clear();
            telemetry.addData("Power", power);
            telemetry.addLine("-------");

            telemetry.update();

            int direction = right? 1 : -1;

            setPowers(power * direction * 0.25, power * direction * -0.25, power * direction * -0.25, power * direction * 0.25);
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

            double power = minPower + (maxPower - minPower) * Math.abs(error / 16d);

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
            setPowersSmoothed(fl, fr, bl, br, 1/50d);
//            telemetryMotorVelocities();
            timer.updateTime();
        }

        while (Math.abs(FL.getPower()) > 0.1) {
            setPowersSmoothed(0, 0, 0, 0, 1/50);
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
