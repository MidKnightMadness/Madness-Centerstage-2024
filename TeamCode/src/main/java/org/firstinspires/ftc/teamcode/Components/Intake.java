package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    // Units are millimeters
    // Drawing:
    // https://docs.google.com/drawings/d/1y0ESeoueBK8GQwIP18MR_x3pGB41Nf1QV6QQpi0wvFI/edit

    final double LENGTH_SERVO_WHEEL = 216.88;
    final double HEIGHT_SERVO = 108.717;

    final double WHEEL_RADIUS = 25.4;

    public DcMotor motor;
    public Servo servo;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intakeMotor");
        servo = hardwareMap.get(Servo.class, "servo");
    }
    void setMotorPower(double power) {
        motor.setPower(power);

    }

    double getIntakeHeight(double servoAngle) {
        return HEIGHT_SERVO - WHEEL_RADIUS - LENGTH_SERVO_WHEEL * Math.cos(servoAngle);
    }


    void setIntakeHeight(double targetHeight, double servoPosition) {
        double servoAngle = servoPosToAngle(servoPosition);
        double currentHeight = getIntakeHeight(servoAngle);
        double heightDifference = targetHeight - currentHeight;
    }

    final int MAX_SERVO_BOUND = 1;
    final int MIN_SERVO_BOUND = 0;
    final int SERVO_RANGE = MAX_SERVO_BOUND - MIN_SERVO_BOUND;

    final double rotationRange = Math.PI / 2;

    double angleToServoPos(double servoPos) {
        return (MAX_SERVO_BOUND - servoPos) / SERVO_RANGE * rotationRange;
    }

    double servoPosToAngle(double angle) {
        return (rotationRange - angle) / rotationRange;
    }

}
