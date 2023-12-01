package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

@TeleOp
public class AutoDeadReckoning extends OpMode {
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;

    Timer timer;

    ButtonToggle a;

    double [] RPMs = {248.7,
            186.5,
            249.1 ,
            186.7};

    double min = RPMs[1];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1] , min / RPMs[2] , min / RPMs[3]};

    @Override
    public void init() {
        timer = new Timer();
        a = new ButtonToggle();

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
    public void loop() {
        if (this.gamepad1.y) {
            setMotorPowers(1, 1, 1, 1);
        }

        if (a.update(gamepad1.a)) {
            driveForwardForTime(2.5, 0.75);
        }

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
        FL.setPower(flPow * 16d/25 * RPMMultipliers[0]);
        FR.setPower(frPow * RPMMultipliers[1]);
        BL.setPower(blPow * 16d/25 * RPMMultipliers[2]);
        BR.setPower(brPow * RPMMultipliers[3]);
    }

    void driveForwardForTime(double seconds, double power) {
        timer.updateTime();
        double startTime = timer.getTime();
        while (timer.getTime() - startTime < seconds) {
            setMotorPowers(power, power, power, power);
            telemetryMotorVelocities();
            timer.updateTime();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    void telemetryMotorVelocities() {
        telemetry.addData("FL RPM", radPSToRPM(FL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("FR RPM", radPSToRPM(FR.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BL RPM", radPSToRPM(BL.getVelocity(AngleUnit.RADIANS)));
        telemetry.addData("BR RPM", radPSToRPM(BR.getVelocity(AngleUnit.RADIANS)));
    }
}
