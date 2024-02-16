package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;

@TeleOp(name = "three")
public class ColorSensorOpMode extends OpMode {
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;

    ColorSensorWrapper colorSensorWrapper1;
    ColorSensorWrapper colorSensorWrapper2;
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;


    ElapsedTime timer;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.get(ColorSensor.class, "cs1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "cs2");

        colorSensorWrapper1 = new ColorSensorWrapper(colorSensor1);
        colorSensorWrapper2 = new ColorSensorWrapper(colorSensor2);

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


        timer = new ElapsedTime();
        timer.startTime();
    }

    public static final double [] FORWARD = {-1.0, 1.0, -1.0, -1.0};
    @Override
    public void loop() {
        if(timer.seconds() < 1){
            FR.setPower(-0.5);
            BR.setPower(0.5);
            BL.setPower(0.5);
            FL.setPower(0.5);
        }

      if(colorSensor2.blue()>100&&colorSensor1.blue()<100){
            FL.setPower(0.5);
            BL.setPower(0.5);

            FR.setPower(0);
            BR.setPower(0);
        }

        else if(colorSensor2.blue()<100&&colorSensor1.blue()>100){
            FR.setPower(-0.5);
            BR.setPower(0.5);

            FL.setPower(0);
            BL.setPower(0);
        }

        if(colorSensor1.blue()>100||colorSensor2.blue()>100){
            FR.setPower(-0.5);
            BR.setPower(0.5);
            BL.setPower(0.5);
            FL.setPower(0.5);
        }








    }
}
