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

    public RangeSensorLocalizer(int detectionDelay, double relX, double relY, HardwareMap hardwareMap, String tag) {
        this.detectionDelay = detectionDelay;
        this.relativeCoords [0] = relX;
        this.relativeCoords [1] = relY;

        distanceSensor = hardwareMap.get(DistanceSensor.class, tag);
    }

    @Override
    double[] getRelCoords(double robotHeading) {
        // Assumes pointing at top or bottom wall (not audience or backstage side)
        // If sensor is pointing down
        if(relativeCoords [0] > 0){ // Assumes right is positive, pointing at bottom wall
            distanceSensor.getDistance(DistanceUnit.MM);
            analog.getVoltage();


        }else{ // Pointing at top wall

        }

        return new double[0];
    }

    @Override
    double[] getCoords(double robotHeading) {
        return new double[0];
    }
}
