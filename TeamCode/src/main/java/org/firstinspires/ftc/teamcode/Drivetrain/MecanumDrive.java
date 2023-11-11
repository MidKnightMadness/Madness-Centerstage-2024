package org.firstinspires.ftc.teamcode.Drivetrain;

import static java.lang.Thread.sleep;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

/*
Configuration:
Control Hub:
Motors:
0: FR
1: BR
2: Center Encoder
3: Right Encoder

Expansion Hub:
Motors:
0:
1:
2: BL
3: FL, Left Encoder
 */

public class MecanumDrive {
    // Motors
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;
    Telemetry telemetry;
    // Static motor power multiplier constants
    // Assumes all motors pointing outwards
    // Forward is side left clockwise, right side counterclockwise
    // Strafing right is FL clockwise, FR counterclockwise, BL counterclockwise, BR clockwise
    // Turning right is FL clockwise, FR clockwise, BL clockwise, BR clockwise

    // FR was being weird
    public static final double [] FORWARD = {-1.0, 1.0, -1.0, -1.0};
    ///forward: -1.0, 1.0, -1.0, -1.0
    public static final double [] RIGHT = {-1.0, -1.0, 1.0, -1.0};
    public static final double [] CLOCKWISE = {-1.0, -1.0, -1.0, 1.0};
    public static final double POWER_MULTIPLIER = 1;

    // Inputs and power constraints
    private double [] motorInputs;

    double [] RPMs = {398.8,
            186.5,
            389.5,
            186};
    double min = RPMs[3];
    double[] RPMMultipliers = { min / RPMs[0], min / RPMs[1] , min / RPMs[2] , min / RPMs[3]};
//    double[] RPMMultipliers = { 1, 1 ,1 , 1};

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.telemetry = telemetry;

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        FL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));
//        FR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));
//        BL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));
//        BR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));

        motorInputs = new double [4];
    }

    // Driving code for TeleOp
    public void normalDrive(double x, double y, double rotation){
        double maxPowerLevel = 0.0;

        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * y) + (RIGHT [i] * x) + (CLOCKWISE [i] * rotation));
            motorInputs[i] *= POWER_MULTIPLIER * RPMMultipliers[i];
//
//            if(Math.abs(motorInputs [i]) > maxPowerLevel){
//                maxPowerLevel = Math.abs(motorInputs [i]);
//            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
//        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]) / 2.0;
//        if(powerEnvelope > 0.2 && maxPowerLevel > 1.0){
//            for(int i = 0; i < 4; i++){
//                motorInputs [i] /= maxPowerLevel;
//            }
//        }

//        telemetry.addData("Power envelope", powerEnvelope);
//        telemetry.addData("Max Power", maxPowerLevel);
//        telemetry.addData("X", x);
//        telemetry.addData("Y", y);

        telemetry.addData("FL", motorInputs [0]);
        telemetry.addData("FR", motorInputs [1]);
        telemetry.addData("BL", motorInputs [2]);
        telemetry.addData("BR", motorInputs [3]);

        telemetry.addData("FL spin", FL.getVelocity());
        telemetry.addData("FR spin", FR.getVelocity());
        telemetry.addData("BL spin", BL.getVelocity());
        telemetry.addData("BR spin", BR.getVelocity());

        setMotorPowers();
    }

    void setMotorPowers() {
        FL.setPower( motorInputs [0]);
        FR.setPower( motorInputs [1]);
        BL.setPower( motorInputs [2]);
        BR.setPower( motorInputs [3]);
    }

    // Built-in ow pass for autonomous purposes
    public double previousX = 0.0;
    public double previousY = 0.0;
    public void FieldOrientedDrive(double x, double y, double rotation, double angle, Telemetry telemetry){ // Angle of front from horizontal right, meant for controller inputs
        double maxPowerLevel = 0.0;

        // Low pass
        double lowPassX = 0.05 * x + 0.95 * previousX;
        double lowPassY = 0.05 * y + 0.95 * previousY;


        // Rotate x and y by negative of angle
        double newX = lowPassX*Math.cos(angle - (Math.PI / 2.0)) + lowPassY*Math.sin(angle - (Math.PI / 2.0));
        double newY = -lowPassX*Math.sin(angle - (Math.PI / 2.0)) + lowPassY*Math.cos(angle - (Math.PI / 2.0));

        // Update low pass previous variables
        previousX = x;
        previousY = y;

        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * newY) + (RIGHT [i] * newX) + (CLOCKWISE [i] * rotation)) * POWER_MULTIPLIER * RPMMultipliers[i] ;

            if(Math.abs(motorInputs [i]) > maxPowerLevel){
                maxPowerLevel = Math.abs(motorInputs [i]);
            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]);
        if(powerEnvelope > 0.2 && maxPowerLevel > 1.0){
            for(int i = 0; i < 4; i++){
                motorInputs [i] /= maxPowerLevel;
            }
        }

//        telemetry.addData("\nPower envelope", powerEnvelope);
//        telemetry.addData("Max Power", maxPowerLevel);
//        telemetry.addData("X", newX);
//        telemetry.addData("Y", newY);
//
//        telemetry.addData("FL", motorInputs [0]);
//        telemetry.addData("FR", motorInputs [1]);
//        telemetry.addData("BL", motorInputs [2]);
//        telemetry.addData("BR", motorInputs [3]);
//        telemetry.addData("Low pass latency", 0.5);

        setMotorPowers();
    }
}
