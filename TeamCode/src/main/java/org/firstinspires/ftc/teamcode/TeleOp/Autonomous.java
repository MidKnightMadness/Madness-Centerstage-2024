//package org.firstinspires.ftc.teamcode.TeleOp;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
//import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
//import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
//
//import org.firstinspires.ftc.teamcode.Utility.Vector2;
//
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//
//@TeleOp(name = "AutonomousPixels")
//public class Autonomous extends OpMode {
//
//    /*
//    NOTES FOR NEXT CODING: if put in two completely different cases, for starting in backdrop and starting in stack, and make a route with three additional coordinates that get inverted for the other way around
//5
//When at backdrop: if (currentTimeMillis()-startTime) > 23) (estimating that 7 seconds max to get pixels and come back):
//Park at backstage
//
//Cycle
//
//Opmode 1 and 2 are team color red -> teamColor = 1
//Opcode 3 and 4 are team color blue -> teamColor = 2
//
//     */
//
//    double[][] targetStates = {
//            //add additional coords for getting 'close enough' for intake and outtake
//            //red
//            //5 total points: Outtake, rightmost (or leftmost), down past trusses, intake stacks
//            //front red also has {36, 12}
//            {84, 12, 0},
//            {120, 36, 0},
//            {84, 12, 0},
//            {12, 12, 0},
//            {12, 36, 0},
//            {36, 12, 0},
//            {84, 12, 0},
//            //and repeat
//            {120, 36, 0}, {84, 12, 0}, {12, 12, 0}, {12, 36, 0}, {36, 12, 0}, {84, 12, 0}, {120, 36, 0}, {84, 12, 0}, {12, 12, 0}, {12, 36, 0}, {36, 12, 0}, {84, 12, 0},
//            //blue starting here
//            //front blue also has {36, 132}
//            {84, 132, 0},
//            {120, 108, 0},
//            {84, 132, 0},
//            {12, 132, 0},
//            {12, 68, 0},
//            {36, 132, 0},
//            {84, 132, 0},
//            //and repeat
//            {84, 132, 0}, {120, 108, 0}, {84, 132, 0}, {12, 132, 0}, {12, 68, 0}, {36, 132, 0}, {84, 132, 0}, {84, 132, 0}, {120, 108, 0}, {84, 132, 0}, {12, 132, 0}, {12, 68, 0}, {36, 132, 0}, {84, 132, 0}
//
//
//    };
//    DistanceSensor distSensor;
//
//    int teamColor = 0; //put this somewhere else; red is 3, blue is 1
//    int numberOfPointsReached = 0;
//    /*if (teamColor ==3)
//
//    {
//        numberOfPointsReached += a lot;
//    }*/
//    MecanumDrive drive;
//    Odometry odometry;
//    PIDDrive PIDDrive;
//
//    ElapsedTime timer;
//    boolean taskComplete = false; //just to make a few things run for now
//    boolean teamPropTask = false;
//    int nextTask = 2; //1 is for intake, and 2 is for outtake
//
//    int stored = 1; //the starting pixel in robot
//
//    @Override
//    public void init() {
//        drive = new MecanumDrive(hardwareMap, telemetry);
//        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));
//        PIDDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);
//        timer = new ElapsedTime();
//        timer.startTime();
//        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
//
//        PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1], targetStates[numberOfPointsReached][2]);
//    }
//
//    @Override
//    public void loop() {
//
//        //while the task at location is complete
//        odometry.updatePosition();
//        PIDDrive.updatePID();
//        if (timer.time() < 23) {
//            if (teamPropTask == false) {
//                PIDDrive.setTargetState(teamProp.x, teamProp.y, teamColor * Math.PI / 2);
//                if (PIDDrive.distanceToTarget < 0.1) {
//                    teamPropTask = true;
//                    intake.setPower(-1);
//                    try {
//                        Thread.sleep(500);
//                    } catch (InterruptedException e) {
//                        throw new RuntimeException(e);
//                    }
//                    intake.setPower(0);
//                }
//
//            } else {
//                if (targetStates[numberOfPointsReached-1][0] == 12 || targetStates[numberOfPointsReached-1][0] == 120) { //arbitrary numbers check if it's one of the points where you run intake or outtake
//                    taskComplete = false;
//                }
//
//                if (taskComplete == false) { //for both intake and outtake
//                    if (nextTask == 1) {//if currently we're in stacks
//                        //intake
//                        //stored++ for each intake
//
//                        taskComplete = true;
//                        nextTask = 2;
//                    }
//                } else {
//                    //outtake
//                    //stored-- each time
//                    if (stored == 0) {
//                        taskComplete = true;
//                        nextTask = 1;
//
//                    }
//                }
//                //help how do i
//
//
//                if (taskComplete == true) {
//                    if (PIDDrive.distanceToTarget < 0.1) { //when we reach the desired point, then our next point is queued
//                        numberOfPointsReached++;
//                    }
//                    //above is the exact same thing but when on backstage
//                    if (numberOfPointsReached < targetStates.length) { //movement
//                            PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1], targetStates[numberOfPointsReached][2]);
//                    }
//                }
//            }
//
//        }else{
//            if (teamColor == 3){
//                PIDDrive.setTargetState(odometry.getXCoordinate(), 12, 0);
//                if (PIDDrive.distanceToTarget < 0.1){
//                    PIDDrive.setTargetState(130, 12, 0);
//                }
//            }
//            else{
//                PIDDrive.setTargetState(odometry.getXCoordinate(), 132, 0);
//                if (PIDDrive.distanceToTarget < 0.1){
//                    PIDDrive.setTargetState(130, 132, 0);
//                }
//
//            }
//
//        }
//    }
//    }
