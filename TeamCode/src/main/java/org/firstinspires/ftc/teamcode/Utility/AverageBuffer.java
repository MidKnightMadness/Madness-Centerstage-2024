package org.firstinspires.ftc.teamcode.Utility;

import java.util.ArrayList;

public class AverageBuffer {

    ArrayList<Double> buffer;
    final int bufferSize;
    double value;

    public AverageBuffer(int bufferSize) {
        buffer = new ArrayList<Double>();
        this.bufferSize = bufferSize;
    }

    public void update(double value) {
        buffer.add(value);

        if (buffer.size() >= bufferSize) {
            this.value = getAverage();
            buffer.clear();
        }
    }

    private double getAverage() {
        double sum = 0;
        for (int i = 0; i < buffer.size(); i++) {
            sum += buffer.get(i);
        }

        return sum / buffer.size();
    }

    public double getValue() {
        return value;
    }

}
