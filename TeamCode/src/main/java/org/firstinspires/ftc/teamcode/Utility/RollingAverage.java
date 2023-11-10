package org.firstinspires.ftc.teamcode.Utility;

public class RollingAverage {
    int count;
    double average;
    double sum;
    String name;

    public RollingAverage() {

    }

    public RollingAverage(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }


    public void update(double value) {
        sum += value;
        count++;
        average = sum / (double) count;
    }

    public double getAverage() {
        return average;
    }

    public void reset() {
        count = 0;
        average = 0;
        sum = 0;
    }
}
