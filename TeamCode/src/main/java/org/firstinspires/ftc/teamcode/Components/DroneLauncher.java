package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DroneLauncher {
    Servo servo;

    final double min = 0.8245;//1
    final double max = 0.9265;//0
    final double difference = max-min;


    public DroneLauncher(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);

        //set the initial position
        servo.setPosition(0.9265);//0 of the start position
    }

    public void setPostion(double position) {
        if(position<1&&position>0){//within the range of 0 to
            double x = position * difference;
            servo.setPosition(max-x);
         }
    }//can use the position of the servo to set the motor to


    public void lock(){
        servo.setPosition(max);
    }

    public void open(){
        servo.setPosition(min);
    }







}
