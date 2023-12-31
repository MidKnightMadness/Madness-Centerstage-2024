package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Timer {
    public ElapsedTime elapsedTime;
    private double lastTime;
    private double deltaTime;
    private double currentTime = lastTime;

    public Timer() {
        this.elapsedTime = new ElapsedTime();
        lastTime = elapsedTime.startTime();
    }

    public Timer(ElapsedTime elapsedTime) {
        this.elapsedTime = elapsedTime;
    }

    public double updateTime() {
        currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        return currentTime;
    }

    public double getTime() {
        return currentTime;
    }

    public double getDeltaTime() {
        return deltaTime;
    }

    public double getStartTime() {
        return elapsedTime.startTime() / 10000d;
    }
}