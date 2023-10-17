package org.firstinspires.ftc.teamcode.Localization;

public class RangeSensorLocalizer extends Localizer{
    public RangeSensorLocalizer(int detectionDelay, double relX, double relY) {
        this.detectionDelay = detectionDelay;
        this.relativeCoords [0] = relX;
        this.relativeCoords [1] = relY;


    }

    @Override
    double[] getRelCoords() {
        return new double[0];
    }

    @Override
    double[] getCoords() {
        return new double[0];
    }
}
