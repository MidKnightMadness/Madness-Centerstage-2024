package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.RollingAverage;

@TeleOp(name = "Drivetrain wheel Calibration")
public class WheelCalibration extends OpMode {
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;

    double [] RPMs = {410.1,
            212.6,
            393.8,
            206.7};
    double min = RPMs[3];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1] , min / RPMs[2] , min / RPMs[3]};
//    double[] RPMMultipliers = { 1, 1 ,1 , 1};

    RollingAverage[] wheelRPMS = {
        new RollingAverage("FL"),
        new RollingAverage("FR"),
        new RollingAverage("BL"),
        new RollingAverage("BR"),
    };

    double rightSideMultiplier = 1;

    DcMotorEx[] motors;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[] { FL, FR, BL, BR };
    }

    @Override
    public void loop() {
        setAdjustedPowers(0);

        if (this.gamepad1.x) {
            setAdjustedPowers(1);
            updateWheelRPMS();
        }

        if (this.gamepad1.a) {
            setPowersDirectly(1);
        }

        if (gamepad1.dpad_up) {
            rightSideMultiplier += 0.0005;
        }
        if (gamepad1.dpad_down) {
            rightSideMultiplier -= 0.0005;
        }
    }
    void setAdjustedPowers(double power) {
        for (int i = 0; i < motors.length; i++) {
            double adjustPower = power * RPMMultipliers[i];
            motors[i].setPower(adjustPower);
            telemetry.addData(wheelRPMS[i].getName(), adjustPower);
        }
    }

    void setPowersDirectly(double power) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power);
            telemetry.addData(wheelRPMS[i].getName(), power);
        }
    }

    void updateWheelRPMS() {
        for (int i = 0; i < motors.length; i++) {
            wheelRPMS[i].update(velocityRadiansToRPM(motors[i].getVelocity(AngleUnit.RADIANS)));
        }
    }

    void logAverages() {
        for (int i = 0; i < motors.length; i++) {
            telemetry.addLine(String.format("%s Motor: %.3f RPM", wheelRPMS[i].getName(), wheelRPMS[i].getAverage()));
        }
    }

    double velocityRadiansToRPM(double velocity) {
        return velocity * 30 / Math.PI;
    }

}