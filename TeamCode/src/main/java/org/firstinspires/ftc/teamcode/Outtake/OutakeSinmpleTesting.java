package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "outtake servo testing")
public class OutakeSinmpleTesting extends OpMode {

    public Servo servo;
    public ElapsedTime timer;

    // Conversion constants
    private final double RADIANS_PER_TICK = Math.PI * 1.5;
    private double speed = Math.PI; // Radians per second


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        if(gamepad1.a==true){
            telemetry.addLine("Servo a is pressed, scoring position");
            while(servo.getPosition() > 0.6){
                servo.setPosition(servo.getPosition() - 0.01);
                try {
                    Thread.sleep((long) Math.round(10.0 * RADIANS_PER_TICK / speed));
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        else{
            while(servo.getPosition() < 1.0){
                servo.setPosition(servo.getPosition() + 0.01);
                try {
                    Thread.sleep((long) Math.round(10.0 * RADIANS_PER_TICK / (1.5 * speed)));
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }


        telemetry.update();
    }
}
