package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RangeSensorLocalizer extends Localizer{
    // Specific fields for range sensor
    double angleToPerpendicular = 0.0; // Radians, for if the distance sensor does not point directly off to the side; so this is generally 0
    public DistanceSensor distanceSensor;
    public AnalogInput analog;
    private double robotHeading = 0.0;

    public RangeSensorLocalizer(int detectionDelay, double relX, double relY, HardwareMap hardwareMap, String tag) { // Time delay for detection, relative location on robot coodinates, hardwaremap, and name
        this.detectionDelay = detectionDelay;
        this.relativeCoords [0] = relX;
        this.relativeCoords [1] = relY;

        distanceSensor = hardwareMap.get(DistanceSensor.class, tag);
    }

    @Override
    double[] getRelCoords(double robotHeading, double currentX, double currentY) {
        // Assumes pointing at top or bottom wall (not audience or backstage side)
        this.robotHeading = robotHeading;
        this.robotHeading %= Math.PI * 2.0;
        if(this.robotHeading < 0.0){
            this.robotHeading += Math.PI * 2.0;
        }

        if((relativeCoords [0] > 0 && (this.robotHeading > 3.0 * Math.PI / 2.0 || this.robotHeading < Math.PI / 2.0)) ||
                (relativeCoords [0] < 0 && (this.robotHeading >= Math.PI / 2.0 && this.robotHeading <= 3.0 * Math.PI / 2.0))){ // Pointing at bottom wall

            if(relativeCoords [0] > 0){
                return new double [] {0.0, Math.cos(this.robotHeading) * distanceSensor.getDistance(DistanceUnit.MM) / 1000.0};
            }else{
                return new double [] {0.0, - Math.cos(this.robotHeading) * distanceSensor.getDistance(DistanceUnit.MM) / 1000.0};
            }

        }else if((relativeCoords [0] > 0 && (this.robotHeading >= Math.PI / 2.0 && this.robotHeading <= 3.0 * Math.PI / 2.0)) ||
                (relativeCoords [0] < 0 && (this.robotHeading > 3.0 * Math.PI / 2.0 || this.robotHeading < Math.PI / 2.0))){ // Pointing at top wall

            if(relativeCoords [0] > 0){
                return new double [] {0.0, 12.0 * 12.0 + Math.cos(this.robotHeading) * distanceSensor.getDistance(DistanceUnit.MM) / 1000.0};
            }else{
                return new double [] {0.0, 12.0 * 12.0 - Math.cos(this.robotHeading) * distanceSensor.getDistance(DistanceUnit.MM) / 1000.0};
            }
        }else{
            return null;
        }
    }


    @Override
    double[] getCoords(double robotHeading, double currentX, double currentY) {
        this.sensorCoords = getRelCoords(robotHeading, currentX, currentY);
        if(this.sensorCoords != null) {
            return new double[]{ // Rotates negative of displacement vector on robot coords and adds it to coordinates of sensor
                    -Math.cos(robotHeading) * this.relativeCoords[0] - Math.sin(robotHeading) * this.relativeCoords[1] + this.sensorCoords[0],
                    Math.sin(robotHeading) * this.relativeCoords[0] - Math.cos(robotHeading) * this.relativeCoords[1] + this.sensorCoords[1]
            };
        }else{
            return null;
        }
    }
}
