package org.firstinspires.ftc.teamcode.Testing.SplineDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp (name = "Basic Servo Testing", group = "testing")
public class SingleServoTest extends OpMode{
    Servo servo;
    ServoController controller;
    double targetTicks = 0.50; // Set to start in middle of range

    // Launcher
//    double [] bounds = {0.65, 0.925}; // locked, open

    // Box Servos
//    double [] bounds = {0.8615, 0.578}; // right, left
    // Right Side
    double [] bounds = {0.25, 1.0d}; // outboard, inboard
    // Left Side
//    double [] bounds = {0.85, 0.1}; // outboard, inboard

    // Outtake elbow servos
    // Right Side
//    double [] bounds = {0.85, 0.1}; // outboard, inboard
    // Left Side
//    double [] bounds = {0.85, 0.1}; // outboard, inboard

    // Box Servo
    double centerTicks = 0.7160;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Center box servo");
        controller = servo.getController();
    }

    @Override
    public void loop() {
        telemetry.addData("Target Pos", servo.getPosition());
        telemetry.addData("Center Pos", centerTicks);
        telemetry.addData("pwm", controller.getPwmStatus());
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

    }
}
