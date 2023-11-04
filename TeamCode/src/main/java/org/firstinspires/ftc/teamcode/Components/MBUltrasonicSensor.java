package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MBUltrasonicSensor {

    AnalogInput analogInput;
    int bufferSize;


    public MBUltrasonicSensor(AnalogInput input, int bufferSize){
        this.analogInput = input;
    }

    public MBUltrasonicSensor(HardwareMap hardwareMap, String name, int bufferSize) {
        this.analogInput = hardwareMap.get(AnalogInput.class, name);
    }



}
