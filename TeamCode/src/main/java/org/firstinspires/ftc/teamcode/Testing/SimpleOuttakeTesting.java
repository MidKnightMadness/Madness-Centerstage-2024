package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "outtake servo testing")
public class SimpleOuttakeTesting extends OpMode {
    final double LEFT_BOUND = 0.5;
    final double RIGHT_BOUND = 0;
    final double MIDDLE = 0.25;

    public Servo servo;
    public ElapsedTime timer;

    // Conversion constants
    private final double RADIANS_PER_TICK = Math.PI;
    private double speed = Math.PI; // Radians per second


    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "outtake_servo");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            telemetry.addLine("Servo a is pressed, scoring position");
            while(servo.getPosition() > LEFT_BOUND){
                servo.setPosition(servo.getPosition() - 0.01);
                try {
                    Thread.sleep((long) Math.round(10.0 * RADIANS_PER_TICK / speed));
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
        else{
            while(servo.getPosition() < RIGHT_BOUND){
                servo.setPosition(servo.getPosition() + 0.01);
                try {
                    Thread.sleep((long) Math.round(10.0 * RADIANS_PER_TICK / (speed)));
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }

        servo.setPosition(MIDDLE);


        telemetry.update();
    }
}
