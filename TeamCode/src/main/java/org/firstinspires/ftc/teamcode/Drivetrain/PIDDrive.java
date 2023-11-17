package org.firstinspires.ftc.teamcode.Drivetrain;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDDrive{
    // 6-14-23 Tuned for Mayhem '22-'23 bot w/ only bottom half,
    private Odometry odometry;
    private Telemetry telemetry;

    // Auxillary variables (low pass, PID, etc...), currently hardcoded since no real need to do otherwise
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    //    private TelemetryPacket telemetryPacket;

    // PID for spline driving
//    double [] P = {0.2, 0.2, -1.0};
//    double [] I = {0.1, 0.1, -0.15};
//    final double integralDecay = 0.95;
//    double [] D = {0.35, 0.35, 0.25};
//    double [] D2 = {0.0, 0.0, 0.0};
//    double [] cumulativeError = {0.0, 0.0, 0.0};

    // For non-spline-path driving - Mayhem chassis 2022
//    double [] P = {0.15, 0.15, -1.0};
//    double [] I = {0.4, 0.4, -0.075};
//    final double integralDecay = 0.95;
//    double [] D = {0.1, 0.1, -0.05};
//    double [] D2 = {0.75, 0.75, -0.05};
//    double [] cumulativeError = {0.0, 0.0, 0.0};


    // For non-spline-path driving - Madness robot 2022
//    double [] P = {0.4, 0.4, 0.0};
//    double [] I = {0.1, 0.1, 0.0};
//    final double integralDecay = 0.9;
//    double [] D = {0.05, 0.06, 0.0};
//    double [] D2 = {0.0, 0.0, 0.0};
//    double [] cumulativeError = {0.0, 0.0, 0.0};

    // For spline path driving - Madness robot 2022
//    double [] P = {0.3, 0.3, 0.0};
//    double [] I = {0.1, 0.1, 0.0};
//    final double integralDecay = 0.9;
//    double [] D = {0.0, 0.0, 0.0};
//    double [] D2 = {0.0, 0.0, 0.0};
//    double [] cumulativeError = {0.0, 0.0, 0.0};

    // For non-spline-path driving - Madness robot 2023
    public double [] P = {0.62576378, 0.62576378, -3.3};
    public double [] I = {0.03234029, 0.03234029, -.15};
    final double integralDecay = 0.9;
    public double [] D = {0.0699999, 0.0699999, 0.0};
    public double [] D2 = {0.0, 0.0, 0.0};
    double [] cumulativeError = {0.0, 0.0, 0.0};


    // Navigational variables
    double [] delta = {0.0, 0.0, 0.0};
    double [] targetState = {0.0, 0.0, 0.0};
    public double distanceToTarget = 0.0;
    double lastDistanceToTarget = 0.0;
    double initialDistanceToTarget = 0.0;
    double integralTermMultiplier = 0.0;

    public  PIDDrive (Odometry odometry, double targetX, double targetY, double targetRadians, Telemetry telemetry){
        this.odometry = odometry;
        this.telemetry = telemetry;

        this.targetState [0] = targetX;
        this.targetState [1] = targetY;
        this.targetState [2] = targetRadians;

        initialDistanceToTarget = (targetState [0] - odometry.getXCoordinate()) * (targetState [0] - odometry.getXCoordinate());
        initialDistanceToTarget += (targetState [1] - odometry.getYCoordinate()) * (targetState [1] - odometry.getYCoordinate());
        initialDistanceToTarget = Math.sqrt(initialDistanceToTarget);
    }

    public double [] updatePID(){ // Update as often as possible, preferrably every tick
        // Since "delta" takes into consideration all the PID coefficients and is directly inputted into the drive code, distance to target calculated seperately from delta
        distanceToTarget = (targetState [0] - odometry.getXCoordinate()) * (targetState [0] - odometry.getXCoordinate());
        distanceToTarget += (targetState [1] - odometry.getYCoordinate()) * (targetState [1] - odometry.getYCoordinate());
        distanceToTarget = Math.sqrt(distanceToTarget);
        telemetry.addData("Distance To Target", distanceToTarget);

        // UpdateTime is already inside updatePosition method
        odometry.updatePosition();

        // Output data (was used in testing)
        telemetry.addLine("\nupdated14");
        telemetry.addData("Update rate", 1.0 / odometry.deltaTime);
        telemetry.addData("\nPID ======================\nLeft", odometry.leftTicks);
        telemetry.addData("Right", odometry.rightTicks);
        telemetry.addData("Front", odometry.topTicks);

        telemetry.addData("\nx", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());

        telemetry.addData("\nTarget x", this.targetState [0]);
        telemetry.addData("Target y", this.targetState [1]);
        telemetry.addData("Target angle", this.targetState [2] * 180.0 / Math.PI);

        telemetry.addData("\nproportional x gain", (targetState [0] - odometry.getXCoordinate()) * P [0]);
        telemetry.addData("proportional y gain", (targetState [1] - odometry.getYCoordinate()) * P [1]);

        telemetry.addData("\nintegral x gain", cumulativeError [0]);
        telemetry.addData("integral y gain", cumulativeError [1]);
        telemetry.addData("integral angular gain", cumulativeError [2]);

        if(distanceToTarget - lastDistanceToTarget < 0.0) { // If approaching target
            telemetry.addData("\nderivative x gain", odometry.getVelocity().x * D[0]);
            telemetry.addData("derivative y gain", odometry.getVelocity().y * D[0]);
        }else{
            telemetry.addData("\nderivative x gain", -Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0] - Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1]);
            telemetry.addData("derivative y gain", Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0] - Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1]);
        }

        // Proportional component, x and y
        delta [0] = (targetState [0] - odometry.getXCoordinate()) * P [0];
        delta [1] = (targetState [1] - odometry.getYCoordinate()) * P [1];


        // Proportional component, angle
        delta[2] = -(targetState[2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) * P[2];
//        if((targetState[2] - odometry.getRotationRadians() % (Math.PI * 2.0)) < Math.PI * 2.0){ // Rotating counterclockwise
//            delta[2] = -(targetState[2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) * P[2];
//        }else{ // Rotating clockwise
//            delta[2] = (targetState[2] - Math.PI * 2.0 - (odometry.getRotationRadians() % (2.0 * Math.PI))) * P[2];
//        }

        // Integral component independently calculated, then added to delta, since "delta" has P and D components added already
        if(distanceToTarget < 1.0) { // Only adjust within small margin
            integralTermMultiplier = 4.0 * Math.exp(-2.0 * (distanceToTarget) * (distanceToTarget)); // Only activates to correct minute errors
            cumulativeError[0] += integralTermMultiplier * I[0] * delta[0];
            cumulativeError[1] += integralTermMultiplier * I[1] * delta[1];
            cumulativeError[2] += I[2] * ((targetState[2] - (odometry.getRotationRadians() % (2.0 * Math.PI))));
        }

        // Will slow robot more by increasing damping term (P term) on approach to target for x and y
        // Meant to solve issue of integral term building up too much when robot starts far away from target, causing robot overshoot
//        double distanceToTargetChangeRate = (distanceToTarget - lastDistanceToTarget) / odometry.deltaTime;
//        if(distanceToTargetChangeRate < 0.0){ // If approaching target
//            // Profiles d term on approach (jacks up term to increase "braking" gain)
//            double XDerivativeMultiplier = 4.0 * Math.exp(-(distanceToTarget - 5.0) * (distanceToTarget - 6.0) / (initialDistanceToTarget / 2.0)) -
//                    3.0 * Math.exp(-(distanceToTarget) * (distanceToTarget) / 9.0);
//            delta [0] -= (Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0] * XDerivativeMultiplier) + (Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1]);
//            delta [1] -= -(Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0] * XDerivativeMultiplier) + (Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1]);
//        }else{ // If overshooting target
//            // Overshoot handled here (will affect first tick leaving a target point to go to next target)
////            delta [0] -= Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0] + Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1];
////            delta [1] -= -Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0] + Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1];
//        }

        // Without lag, on newer control hub, x and y still relativec to robot
//       Profiles d term on approach (jacks up term to increase "braking" gain)
            double derivativeMultiplier = 1.0; //Math.exp(-(distanceToTarget - 2.0d) * (distanceToTarget - 2.0d) / 4.0d) -
//                    1.4 * Math.exp(-1.4 * (distanceToTarget) * (distanceToTarget));
            delta [0] -= odometry.getVelocity().x * D [0] * derivativeMultiplier;
            delta [1] -= odometry.getVelocity().y * D [1] * derivativeMultiplier;
//        delta [0] -= (Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0]) + (Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1]);
//        delta [1] -= -(Math.sin(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().x * D [0]) + (Math.cos(odometry.getRotationRadians() - Math.PI / 2.0) * odometry.getVelocity().y * D [1]);

        // Turning doesn't have have the same overshooting issues, so just do velocity gain calculations
        delta [2] += odometry.angularVelocity * D [2];

        // Add cumulative error up seperately from PID, since P and D components are reset every tick
        delta [0] += cumulativeError [0];
        delta [1] += cumulativeError [1];
        delta [2] += cumulativeError [2];

        // Since inputting gain values into FieldOrientedDrive (made to take inputs from -1 to 1), need to normalize magnitude of x, y, angle inputs
        double maxInputValue = 0.0;
        delta [0] *= P [0];
        delta [1] *= P [0];
        delta [2] *= P [0];
        for(double num : delta){
            if (Math.abs(num) > maxInputValue) {
                maxInputValue = Math.abs(num);
            }
        }
        if(maxInputValue > 1.00){
            for(int i = 0; i < 3; i++){
                delta [i] *=  1.0 / Math.abs(maxInputValue); // Hardcoded power coefficient to slow down robot during testing
            }
        }

        telemetry.addData("x delta input", delta [0]);
        telemetry.addData("y delta input", delta [1]);
        telemetry.addData("angle delta input", delta [2]);
        telemetry.addLine("==========================");

        // Make sure integral term doesn't go out of control
        cumulativeError [0] *= integralDecay;
        cumulativeError [1] *= integralDecay;
        cumulativeError [2] *= integralDecay;

        // For calculating speed of target approach
        lastDistanceToTarget = distanceToTarget;

        // Return values go into FieldOrientedDrive
        return this.delta;
    }

    public void setTargetState(double targetX, double targetY, double targetRadians) {
        this.targetState[0] = targetX;
        this.targetState[1] = targetY;
        this.targetState[2] = targetRadians;
    }
}