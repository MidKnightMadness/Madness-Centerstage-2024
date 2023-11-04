package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

public class MBUltrasonicSensorWrapper {

    AnalogInput analogInput;
    final int BUFFER_SIZE  = 3;
    AverageBuffer averageBuffer;



    public MBUltrasonicSensorWrapper(AnalogInput input, int bufferSize){
        this.analogInput = input;
        averageBuffer = new AverageBuffer(BUFFER_SIZE);
    }

    public MBUltrasonicSensorWrapper(HardwareMap hardwareMap, String name, int bufferSize) {
        this.analogInput = hardwareMap.get(AnalogInput.class, name);
    }

    public double update(){
        averageBuffer.update(analogInput.getVoltage());
        double distance = (averageBuffer.getValue() - 0.0066512) / 0.0004670;
        return distance;
    }
}
