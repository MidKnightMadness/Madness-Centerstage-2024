package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeBox {

    final double LEFT_BOUND = 0.5;
    final double RIGHT_BOUND = 0;
    final double MIDDLE = 0.25;
    Servo servo;

    public OuttakeBox(Servo servo) {
        this.servo = servo;
    }

    public OuttakeBox(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);

    }

    public void outtakePixelLeft() {

    }

    public void outtakePixelRight() {

    }

    void sleep(long milis) {
        try {
            Thread.sleep(milis);
        } catch (InterruptedException e) {

        }
    }

}
