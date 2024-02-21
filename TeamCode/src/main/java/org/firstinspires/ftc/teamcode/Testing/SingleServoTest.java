package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.Components.ServoPositions.boxServoLeft;
import static org.firstinspires.ftc.teamcode.Components.ServoPositions.boxServoRight;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp (name = "Basic Servo Testing", group = "testing")
public class SingleServoTest extends OpMode{
    Servo servo;
    Servo boxServo;
    DcMotorEx intakeMotor;
    double targetTicks = 0.2; // Set to start in middle of range
    double centerTicks = 0.2;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Right elbow servo");
        boxServo = hardwareMap.get(Servo.class, "Center box servo");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake motor");
    }

    @Override
    public void loop() {
        telemetry.addData("Target Pos", servo.getPosition());
        telemetry.addData("Center Pos", centerTicks);
        telemetry.update();
        servo.setPosition(targetTicks);

        if(gamepad1.dpad_up && !gamepad1.dpad_down){
            if(servo.getPosition() > 0.0 && servo.getPosition() < 1.0){
                targetTicks += 0.0005;
            }
        }else if(!gamepad1.dpad_up && gamepad1.dpad_down){
            if(servo.getPosition() > 0.0 && servo.getPosition() < 1.0){
                targetTicks  -= 0.0005;
            }
        }

        boxServo.setPosition((gamepad1.right_stick_x + 2) * (boxServoLeft - boxServoRight) / 2d + boxServoRight);
    }
}
