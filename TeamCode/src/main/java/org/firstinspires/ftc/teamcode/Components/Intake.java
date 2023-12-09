package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{
    DcMotorEx rollerMotor;
    Servo leftServo;
    Servo rightServo;

    // CalculationVariables
    double [] leftServoPresets = {0.8375, 0.89, 0.893, 0.939, 0.9575};
    double [] rightServoPresets = {0.3075, 0.2425, 0.2223, 0.1605, 0.126};

    public Intake(HardwareMap hardwareMap){
        rollerMotor = hardwareMap.get(DcMotorEx.class, "roller motor");
        leftServo = hardwareMap.get(Servo.class, "left intake servo");
        rightServo = hardwareMap.get(Servo.class, "right intake servo");

        rollerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rollerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rollerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void idle(){
        leftServo.setPosition(leftServoPresets [4]);
        rightServo.setPosition(rightServoPresets [4]);

        this.setMotorPower(0.0);
    }

    public void down(){
        this.setMotorPower(1.0);

        leftServo.setPosition(leftServoPresets [0]);
        rightServo.setPosition(rightServoPresets [0]);
    }

    public void spitPixel(){
        leftServo.setPosition(leftServoPresets [0]);
        rightServo.setPosition(rightServoPresets [0]);

        this.setMotorPower(-0.5); // Subject to change
    }

    public void intakeFromStackOf(int stackSize){
        this.setMotorPower(1.0);

        leftServo.setPosition(leftServoPresets [stackSize - 1]);
        rightServo.setPosition(rightServoPresets [stackSize - 1]);
    }

    public double setMotorPower(double power){
        rollerMotor.setPower(power);
        return power;
    }
}