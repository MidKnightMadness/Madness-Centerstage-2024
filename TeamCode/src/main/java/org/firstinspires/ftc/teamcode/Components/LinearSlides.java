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
    public double [] powerConstants = {0.0, 0.0};
    double correctionConstant = 0.0;
    public final double LENGTH_TOLERANCE = 0.1; // inches

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

    public void setHeight(double targetLength) { // Run every tick when you want motors to go to this
        while (Math.abs(extendedLengths [0] - extendedLengths [1]) < extensionDifferenceTolerance && (extendedLengths [0] + extendedLengths [1]) / 2.0d < LENGTH_TOLERANCE &&
                    leftBounds [0] <= extendedLengths [0] && extendedLengths [0] <= leftBounds [1] &&
                    rightBounds [0] <= extendedLengths [1] && extendedLengths [1] <= rightBounds [1]) {
            // Using left side as leader
            motorLeft.setPower((targetLength - extendedLengths [0]) * powerConstants [0]);
            motorRight.setPower((targetLength - extendedLengths [1]) * powerConstants [1] +
                    (extendedLengths [0] - extendedLengths [1]) * correctionConstant);

            extendedLengths [0] = inPerTickLeft * motorLeft.getCurrentPosition();
            extendedLengths [1] = inPerTickRight * motorRight.getCurrentPosition();
        }
    }

}
