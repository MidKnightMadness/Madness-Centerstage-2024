package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeBox {

    final double LEFT_BOUND = 0.5;
    final double RIGHT_BOUND = 0;
    final double MIDDLE = 0.25;
    Servo servo;
    ElapsedTime timer;

    public OuttakeBox(Servo servo) {
        this.servo = servo;
        timer = new ElapsedTime();
    }

    public OuttakeBox(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
        servo.setPosition(0.5);
    }

    public void outtakePixelLeft() {
        // Temporary:
//        servo.setPosition(LEFT_BOUND);

        timer.reset();
        while(servo.getPosition() > LEFT_BOUND){
            servo.setPosition(LEFT_BOUND);
        }
    }

    public void outtakePixelRight() {
        // Temporary:
//        servo.setPosition(RIGHT_BOUND);

        timer.reset();
        while(servo.getPosition() < RIGHT_BOUND){
            servo.setPosition(RIGHT_BOUND);
        }
    }
    public void outtakePixelMiddle(){
        timer.reset();
        servo.setPosition(MIDDLE);

    }

    void sleep(long milis) {
        try {
            Thread.sleep(milis);
        } catch (InterruptedException e) {

        }
    }

}
