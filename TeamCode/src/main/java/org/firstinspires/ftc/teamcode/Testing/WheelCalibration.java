package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Utility.RollingAverage;

@TeleOp(name = "Drivetrain wheel Calibration")
public class WheelCalibration extends OpMode {
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;

    RollingAverage[] wheelRPMS = {
        new RollingAverage("FL"),
        new RollingAverage("FR"),
        new RollingAverage("BL"),
        new RollingAverage("BR"),
    };

    DcMotorEx[] motors;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new DcMotorEx[] { FL, FR, BL, BR };
    }

    @Override
    public void loop() {
        if (this.gamepad1.x) {
            setPowers(1);
            updateWheelRPMS();
        }
        else {
            setPowers(0);
        }

        if (this.gamepad1.y) {
            for (RollingAverage avg: wheelRPMS) {
                avg.reset();
            }
        }

        logAverages();
    }
    void setPowers(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
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