package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlides {
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;

    // Internal use variables
    public int [] leftBounds = {0, 0}; // Retracted, extended
    public int [] rightBounds = {0, 0}; // Retracted, extended
    public double inPerTickLeft = 0.0;
    public double inPerTickRight = 0.0;
    public double extensionDifferenceTolerance = 0.0;
    public double [] extendedLengths = {0.0, 0.0}; // Left, right

    public LinearSlides(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        resetEncoders();
    }

    public void resetEncoders() {
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setHeight(double height) { // Run every tick when you want motors to go to this
//        while (Math.abs(extendedLengths [0] - extendedLengths [1]) < extensionDifferenceTolerance && ) {
//
//        }
    }

}
