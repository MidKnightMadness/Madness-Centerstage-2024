package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@TeleOp (name = "Basic Servo Testing", group = "Testing")
public class SingleServoTest extends OpMode{
    Servo servo;
    ServoController controller;
    double targetTicks = 0.50; // Set to start in middle of range
    // Launcher
    double [] bounds = {0.65, 0.925}; // locked, open
    //Box Servos
//    double [] bounds = {0.8615, 0.578}; // right, left
//    double [] bounds = {0.25, 1.0d}; // outboard, inboard
//    double [] bounds = {0.85, 0.1}; // outboard, inboard

    // Box Servo
    double centerTicks = 0.7160;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
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
                centerTicks += 0.0005;
            }
        }else if(!gamepad1.dpad_up && gamepad1.dpad_down){
            if(servo.getPosition() > 0.0 && servo.getPosition() < 1.0){
                centerTicks -= 0.0005;
            }
        }

        if(gamepad1.y){
//            targetTicks = bounds [0];
            controller.pwmDisable();
        }else {
            controller.pwmEnable();
            targetTicks = bounds[1];
        }
//        }else{
//            targetTicks = centerTicks;
//        }
    }
}
