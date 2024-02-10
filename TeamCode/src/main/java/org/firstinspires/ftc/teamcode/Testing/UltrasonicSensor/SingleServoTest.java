package org.firstinspires.ftc.teamcode.Testing.UltrasonicSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp (name = "Basic Servo Testing", group = "testing")
public class SingleServoTest extends OpMode{
    Servo servo;
    double targetTicks = 0.2; // Set to start in middle of range
    double centerTicks = 0.2;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "Launcher servo");
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

    }
}
