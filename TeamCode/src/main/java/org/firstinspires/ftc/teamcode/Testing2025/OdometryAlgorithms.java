//package org.firstinspires.ftc.teamcode.Testing2025;
//
//public class OdometryAlgorithms {
//    class LinearOdometryAlgorithm{
//        // Subject to adjustment
//        double inPerTickLeft = 30.0 / 3800d;
//        double inPerTickRight = 30.0 / 3800d;
//        double inPerTickFront= 30.0 / 3800d;
//        // Currently not used, though perceived angle change is generally slightly less than actual when simultaneously strafing
//        double HEADING_CORRECTION = 360d / 359.99;
//
//        // SUbject to change
//        double trackWidth = 12.5;
//        double distanceToFront = 1.5;
//
//        // Tracking variables
//        double deltaRadians = 0.0;
//        double [] deltaCoords = {0.0, 0.0};
//
//        // Thresholds
//        final int tickChangeLimit = 1000;
//        boolean updatePosition = false; // If not moved enough, don't update, rounding errors more prevelant
//
//
//        double perceivedHeading = 0.0;
//        double [] perceivedCoords = {0.0, 0.0};
//
//        int numberOfUpdates = 0;
//
//        public OdometryAlgorithm(double startingX, double startingY, double startingHeading) {
//            deltaRadians = 0.0;
//            deltaCoords = new double [2];
//
//            perceivedHeading = startingHeading;
//            perceivedCoords [0] = startingX;
//            perceivedCoords [1] = startingY;
//        }
//
//        public void updatePosition(int [] deltaTicks/* Left, Right, Center */) {
//            // Check how much is moved, need to move a certain amount to remove int rounding errors
//            updatePosition = (Math.abs(deltaTicks [0]*deltaTicks [0] + deltaTicks [1]*deltaTicks [1] + deltaTicks [2]*deltaTicks [2]) > tickChangeLimit)? true : false;
//
//            if(updatePosition) {
//                // Assume left and right are positive for forward, front is positive for counterclockwise
//                deltaRadians = (deltaTicks [1] * inPerTickRight - deltaTicks [0] * inPerTickLeft) / (trackWidth);
//
//                perceivedHeading += deltaRadians;
//
//                deltaCoords [0] = -Math.sin(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.cos(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//                deltaCoords [1] = Math.cos(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.sin(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//
//                perceivedCoords [0] += deltaCoords [0];
//                perceivedCoords [1] += deltaCoords [1];
//            }
//        }
//    }
//
//    class ArcOdometryAlgorithm{
//        // Algorithm side
//        double inPerTickLeft = 30.0 / 38000d;
//        double inPerTickRight = 30.0 / 38000d;
//        double inPerTickFront= 30.0 / 38000d;
//        double HEADING_CORRECTION = 360d / 359.85;
//
//        double trackWidth = 12.5;
//        double distanceToFront = 1.5;
//
//        double deltaRadians = 0.0;
//        double [] deltaCoords = {0.0, 0.0};
//
//        boolean updatePosition = false; // If not moved enough, don't update, rounding errors more prevelant
//        final int tickChangeLimit = 1000;
//
//        double rX = 0.0; // Radius of turn in sideways direction
//        double rY = 0.0; // Radius of turn in forward direction
//
//        double perceivedSidewaysMovement = 0.0; // To the right side relative to robot
//        double perceivedForwardMovement = 0.0; // Forwards relative to robot
//
//        double realSidewaysMovement = 0.0; // With consideration of traveling in an arc
//        double realForwardMovement = 0.0; // With consideration of traveling in an arc
//
//        double perceivedHeading = 0.0;
//        double [] perceivedCoords = {0.0, 0.0};
//
//        int numberOfUpdates = 0;
//
//        RobotSimulation simulator;
//        double lastIMUAngle = 0.0;
//
//        public ArcOdometryAlgorithm(double startingX, double startingY, double startingHeading, RobotSimulation simulator) {
//            deltaRadians = 0.0;
//            deltaCoords = new double [2];
//
//            perceivedHeading = startingHeading;
//            perceivedCoords [0] = startingX;
//            perceivedCoords [1] = startingY;
//
//            this.simulator = simulator;
//            lastIMUAngle = simulator.robotHeading;
//        }
//
//        public void updatePosition(int [] deltaTicks/* Left, Right, Center */) {
//
//            // Check how much is moved, need to move a certain amount to remove int rounding errors
//            updatePosition = (Math.abs(deltaTicks [0]*deltaTicks [0] + deltaTicks [1]*deltaTicks [1] + deltaTicks [2]*deltaTicks [2]) > tickChangeLimit)? true : false;
//
//            if(updatePosition) {
//                deltaRadians = (deltaTicks [1] * inPerTickRight - deltaTicks [0] * inPerTickLeft) / (trackWidth);
//                perceivedHeading += deltaRadians;
//
//                perceivedSidewaysMovement = (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront);
//                perceivedForwardMovement = (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//
//                if(Math.abs(deltaRadians) != 0 && Math.abs(deltaRadians) < 0.1 && Math.sqrt(perceivedSidewaysMovement*perceivedSidewaysMovement + perceivedForwardMovement*perceivedForwardMovement) < 0.5) {
//
//
//                    rX = perceivedSidewaysMovement / deltaRadians; // rY is sideways
//                    rY = perceivedForwardMovement / deltaRadians; // rX is forwards and backwards
//
//                    realSidewaysMovement = rY * (Math.cos(deltaRadians) - 1d) + rX * Math.sin(deltaRadians);
//                    realForwardMovement = rX * (1d - Math.cos(deltaRadians)) + rY * Math.sin(deltaRadians);
//
//                    deltaCoords [0] = Math.cos(perceivedHeading) * realForwardMovement - Math.sin(perceivedHeading) * realSidewaysMovement;
//                    deltaCoords [1] = Math.sin(perceivedHeading) * realForwardMovement + Math.cos(perceivedHeading) * realSidewaysMovement;
//
//                    perceivedCoords [0] += deltaCoords [0];
//                    perceivedCoords [1] += deltaCoords [1];
//                } else {
//                    deltaCoords [0] = -Math.sin(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.cos(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//                    deltaCoords [1] = Math.cos(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.sin(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//
//                    perceivedCoords [0] += deltaCoords [0];
//                    perceivedCoords [1] += deltaCoords [1];
//                }
//            }
//        }
//    }
//
//    class ArcOdometryAlgorithmForTwoWheels{ // If one side encoder is disabled
//        // Algorithm side
//        double inPerTickLeft = 30.0 / 38000d;
//        double inPerTickRight = 30.0 / 38000d;
//        double inPerTickFront= 30.0 / 38000d;
//        double HEADING_CORRECTION = 360d / 359.85;
//
//        double trackWidth = 12.5;
//        double distanceToFront = 1.5;
//
//        double lastPerceivedHeading = 0.0;
//        double deltaRadians = 0.0;
//        double [] deltaCoords = {0.0, 0.0};
//
//        boolean updatePosition = false; // If not moved enough, don't update, rounding errors more prevelant
//        final int tickChangeLimit = 500;
//
//        double rX = 0.0; // Radius of turn in sideways direction
//        double rY = 0.0; // Radius of turn in forward direction
//
//        double perceivedSidewaysMovement = 0.0; // To the right side relative to robot
//        double perceivedForwardMovement = 0.0; // Forwards relative to robot
//
//        double realSidewaysMovement = 0.0; // With consideration of traveling in an arc
//        double realForwardMovement = 0.0; // With consideration of traveling in an arc
//
//        double perceivedHeading = 0.0;
//        double [] perceivedCoords = {0.0, 0.0};
//
//        int numberOfUpdates = 0;
//
//        RobotSimulation simulator;
//        double lastIMUAngle = 0.0;
//
//        public ArcOdometryAlgorithmForTwoWheels(double startingX, double startingY, double startingHeading, RobotSimulation simulator) {
//            deltaRadians = 0.0;
//            deltaCoords = new double [2];
//
//            perceivedHeading = startingHeading;
//            perceivedCoords [0] = startingX;
//            perceivedCoords [1] = startingY;
//
//            this.simulator = simulator;
//            lastIMUAngle = simulator.robotHeading;
//        }
//
//        public void updatePosition(int [] deltaTicks/* Left, Right, Center */) {
//
//            // Check how much is moved, need to move a certain amount to remove int rounding errors
//            updatePosition = (Math.abs(deltaTicks [0]*deltaTicks [0] + deltaTicks [2]*deltaTicks [2]) > tickChangeLimit)? true : false;
//
//            if(updatePosition) {
//                perceivedHeading = simulator.robotHeading;
//                deltaRadians = perceivedHeading - lastPerceivedHeading;
//                lastPerceivedHeading = perceivedHeading;
//
//                perceivedSidewaysMovement = (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront);
//                perceivedForwardMovement = (deltaTicks [0] * inPerTickFront - deltaRadians * trackWidth / 2d);
//
//
//                if(Math.abs(deltaRadians) != 0 && Math.abs(deltaRadians) < 0.1 && Math.sqrt(perceivedSidewaysMovement*perceivedSidewaysMovement + perceivedForwardMovement*perceivedForwardMovement) < 0.5) {
//
//
//                    rX = perceivedSidewaysMovement / deltaRadians; // rY is sideways
//                    rY = perceivedForwardMovement / deltaRadians; // rX is forwards and backwards
//
//                    realSidewaysMovement = rY * (Math.cos(deltaRadians) - 1d) + rX * Math.sin(deltaRadians);
//                    realForwardMovement = rX * (1d - Math.cos(deltaRadians)) + rY * Math.sin(deltaRadians);
//
//                    deltaCoords [0] = Math.cos(perceivedHeading) * realForwardMovement - Math.sin(perceivedHeading) * realSidewaysMovement;
//                    deltaCoords [1] = Math.sin(perceivedHeading) * realForwardMovement + Math.cos(perceivedHeading) * realSidewaysMovement;
//
//                    perceivedCoords [0] += deltaCoords [0];
//                    perceivedCoords [1] += deltaCoords [1];
//                } else {
//                    deltaCoords [0] = -Math.sin(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.cos(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//                    deltaCoords [1] = Math.cos(perceivedHeading) * (deltaTicks [2] * inPerTickFront - deltaRadians * distanceToFront) + Math.sin(perceivedHeading) * (deltaTicks [1] * inPerTickRight + deltaTicks [0] * inPerTickLeft) / 2d;
//
//                    perceivedCoords [0] += deltaCoords [0];
//                    perceivedCoords [1] += deltaCoords [1];
//                }
//            }
//        }
//
//    }
