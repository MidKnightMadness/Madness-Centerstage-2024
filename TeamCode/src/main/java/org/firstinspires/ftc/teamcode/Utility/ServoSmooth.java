package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ServoSmooth {

    boolean parametersSet;
    double startTime;
    Timer timer;

    Servo servo;

    double startPosition;
    double targetPosition;

    public ServoSmooth(Servo servo) {
        timer = new Timer();
        this.servo = servo;
    }


    public boolean setServoPosition(double startingPosition, double targetPosition, double time, Telemetry telemetry) {
        parametersSet = startingPosition == this.startPosition && targetPosition == this.targetPosition;

        if (!parametersSet) {
            startTime = timer.updateTime();
            startPosition = startingPosition;
            this.targetPosition = targetPosition;
            parametersSet = true;
        }

        double deltaTime = timer.updateTime() - startTime;

        telemetry.addData("dt", deltaTime);
        telemetry.addData("sp", startPosition);
        telemetry.addData("tp", this.targetPosition);

        if (deltaTime >= time) {
            servo.setPosition(targetPosition);
        }

        servo.setPosition(startPosition + (targetPosition - startPosition) * deltaTime / time);
        return false;
    }

    public boolean setServoPosition(double targetPosition, double time, Telemetry telemetry) {
        if (!parametersSet) {
            startTime = timer.updateTime();
            startPosition = servo.getPosition();
            parametersSet = true;
        }

        double deltaTime = timer.updateTime() - startTime;

        telemetry.addData("dt", deltaTime);
        telemetry.addData("sp", startPosition);
        telemetry.addData("tp", targetPosition);

        if (deltaTime >= time) {
            servo.setPosition(targetPosition);
            parametersSet = false;
            return true;
        }

        servo.setPosition(startPosition + (targetPosition - startPosition) * deltaTime / time);
        return false;
    }
}
