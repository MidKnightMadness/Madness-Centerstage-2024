package org.firstinspires.ftc.teamcode.Localization;


public abstract class Localizer {
    // Template fields, copy over to use case
    public int detectionDelay = 0; // Delay of data input in milliseconds
    double[] relativeCoords = {0.0, 0.0}; // x, y of target relative to robot x and y coordinates, treats outtake facing direction as +y, 90˚ right of that as +x
    double [] sensorCoords = {0.0, 0.0}; // Location of sensor on field

    abstract double[] getRelCoords(double robotHeading, double currentX, double currentY);
    abstract double[] getCoords(double robotHeading, double currentX, double currentY);
}



