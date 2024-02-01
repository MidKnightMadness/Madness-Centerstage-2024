package org.firstinspires.ftc.teamcode.Testing.UltrasonicSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp (name = "Single Servo Testing", group = "testing")
public class SingleServoTest extends OpMode{
    Servo servo;
    double targetTicks = 0.5; // Set to start in middle of range

    @Override
    public void init() {
            servo = hardwareMap.get(Servo.class, "Right wrist servo");
    }

    @Override
    public void loop() {
        telemetry.addData("Target Pos", servo.getPosition());
        telemetry.update();
        servo.setPosition(targetTicks);

        if(gamepad1.dpad_up && !gamepad1.dpad_down){
            if(servo.getPosition() >= 0.0 && servo.getPosition() <= 1.0){
                targetTicks += 0.0005;
            }
        }else if(!gamepad1.dpad_up && gamepad1.dpad_down){
            if(servo.getPosition() >= 0.0 && servo.getPosition() <= 1.0){
                targetTicks  -= 0.0005;
            }
        }

    }
}
