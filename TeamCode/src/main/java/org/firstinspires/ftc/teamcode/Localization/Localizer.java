package org.firstinspires.ftc.teamcode.Localization;


public abstract class Localizer {
    // Template fields, copy over to use case
    public int detectionDelay = 0; // Delay of data input in milliseconds
    double[] robotCoords = {0.0, 0.0}; // x, y coordinates relative to robot, treats outtake facing direction as +y, 90˚ right of that as +x
    double[] relativeCoords = {0.0, 0.0}; // x, y of target relative to robot x and y coordinates, combines multiple channels for camera
    double[] delta = {0.0, 0.0}; // coordinate with field x and y axes from target to robot
    double[] targetCoords = {0.0, 0.0}; // coordinates of target on field

    abstract double[] getRelCoords(double robotHeading, double currentX, double currentY);
    abstract double[] getCoords(double robotHeading);
}



