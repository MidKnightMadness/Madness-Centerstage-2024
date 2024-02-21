package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;

public class GeneralSensorWrapper {
    //takes in the color sensor values
    int BUFFER_SIZE = 5;
    AverageBuffer averageBuffer;


    public GeneralSensorWrapper(int bufferSize){
        this.BUFFER_SIZE = bufferSize;
        averageBuffer = new AverageBuffer(bufferSize);
    }

    public double update(double value){
        averageBuffer.update(value);
        return averageBuffer.getValue();
    }

    public void clear(){
        averageBuffer.clear();
    }
}
