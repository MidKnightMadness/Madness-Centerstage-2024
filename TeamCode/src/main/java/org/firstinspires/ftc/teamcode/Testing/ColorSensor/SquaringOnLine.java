package org.firstinspires.ftc.teamcode.Testing.ColorSensor;

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
import org.firstinspires.ftc.teamcode.Testing.ColorSensor.LinesEnum.lineColor;
@TeleOp(name = "Squaring Testing")
public class SquaringOnLine extends OpMode {
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;

    lineColor lineColor = LinesEnum.lineColor.Red;
    public lineColor getLineColor(){
        return this.lineColor;
    }
    public void setLineColor(lineColor color){
        this.lineColor = color;
    }

    public double RedMin = 0.6;
    public double RedMax = 1;
    public double BlueMin = 0.6;
    public double BlueMax = 1;

    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;



    ElapsedTime timer;

    @Override
    public void init() {
        colorSensor1 = hardwareMap.get(ColorSensor.class, "Left color sensor");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "Right color sensor");


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

    public void setMotorPowers(double FLP, double FRP, double BLP, double BRP){
        FL.setPower(FLP);
        FR.setPower(FRP);
        BL.setPower(BLP);
        BR.setPower(BRP);
    }
    public static final double [] FORWARD = {-1.0, 1.0, -1.0, -1.0};
    boolean leftCompleted = false;
    boolean rightCompleted = false;
    @Override
    public void loop() {
        boolean leftSide = compareColor(colorSensor1);
        boolean rightSide = compareColor(colorSensor2);
        //assuming can drive forward and line is in front
        if(rightSide==false && leftSide==false){
            setMotorPowers(-0.1,0.1, -0.1, -0.1);//forward
        }
        else if(rightSide==false && leftSide == true){
            setMotorPowers(0.1, 0.15, 0.1, -0.15);//turn right
            leftCompleted = true;
            rightCompleted = true;
        }
        else if(rightSide==true && leftSide == false){
            setMotorPowers(-0.15, 0.1, -0.15, -0.1);//turn right
            leftCompleted = true;
            rightCompleted = true;
        }

        else{//both sides need to be on line
            setMotorPowers(0,0,0,0);
        }


    }

    public boolean compareColor(ColorSensor colorSensor){
        if(colorSensor.red()>RedMin && colorSensor.red()<RedMax){
            this.setLineColor(lineColor.Red);
            return true;
        }

        else if(colorSensor.blue()>BlueMin && colorSensor.blue()<BlueMax){
            this.setLineColor(lineColor.Blue);
            return true;
        }

        return false;
    }





}