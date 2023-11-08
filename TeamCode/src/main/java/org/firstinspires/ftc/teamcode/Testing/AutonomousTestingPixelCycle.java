package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain.IndepDrivetrain;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;

import org.firstinspires.ftc.teamcode.Localization.Localizer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp(name = "AutonomousPixels")
public class AutonomousTestingPixelCycle extends OpMode {

    /*
    NOTES FOR NEXT CODING: if put in two completely different cases, for starting in backdrop and starting in stack, and make a route with three additional coordinates that get inverted for the other way around
5
When at backdrop: if (currentTimeMillis()-startTime) > 23) (estimating that 7 seconds max to get pixels and come back):
Park at backstage

Cycle

Opmode 1 and 2 are team color red -> teamColor = 1
Opcode 3 and 4 are team color blue -> teamColor = 2

     */

    double[][] targetStates = {
            //red
            //4 total points: Outtake, rightmost (or leftmost), down past trusses, intake stacks
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            //blue starting here
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0},
            {0, 0, 0}


    };
    DistanceSensor distSensor;

    int teamColor = 0; //put this somewhere else; red is 1, blue is 0
    int numberOfPointsReached = 0;
    /*if (teamColor ==2)

    {
        numberOfPointsReached += 8;
    }*/


    int releasedNum = 0; //arbitrary value for outtake


    //Cycle
    MecanumDrive drive;
    Odometry odometry;
    PIDDrive PIDDrive;

    ElapsedTime timer;

    boolean taskComplete = false; //just to make a few things run for now
    boolean overriding = false;
    int nextTask = 1; //1 is for intake, and 2 is for outtake
    int intook = 0;
    int outtook = 0;
    int outTakeMax = 1; //the starting pixel in robot

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));
        PIDDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);
        timer = new ElapsedTime();
        timer.startTime();
        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");

        PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1], targetStates[numberOfPointsReached][2]);
    }

    @Override
    public void loop() {

        //while the task at location is complete

        if (targetStates[numberOfPointsReached][0] == 0 ||targetStates[numberOfPointsReached][0] == 1){ //arbitrary numbers check if it's one of the points where you run intake or outtake

        }

        if (taskComplete == false) { //for both intake and outtake
            if (nextTask == 1) {//if currently we're in backdrop

                //intake
                //outTakeMax++ for each intake
                if (intook == 2){
                    intook = 0;
                    taskComplete = true;
                    nextTask = 2;
                }
            }
            else{
                //outtake
                if (outtook == outTakeMax){
                    outtook = 0;
                    outTakeMax = 0;
                    taskComplete =true;
                    nextTask = 1;

                }
            }
             //help how do i
        }


        if (taskComplete == true) {
            odometry.updatePosition();
            PIDDrive.updatePID();
            if (PIDDrive.distanceToTarget < 0.1) { //when we reach the desired point, then our next point is queued
                numberOfPointsReached++;
            }
            if (numberOfPointsReached % 2 != 0 /*on the pixel stacks, change this later */ && distSensor.getDistance(DistanceUnit.INCH) + 0.1 > 12 || distSensor.getDistance(DistanceUnit.INCH) - 0.1 < 12) { //change this later
                PIDDrive.setTargetState(12 - distSensor.getDistance(DistanceUnit.INCH), odometry.getYCoordinate(), 0);
                overriding = true;
            }
            if (numberOfPointsReached % 2 != 0 /*on the backstage stacks, change this later */ && distSensor.getDistance(DistanceUnit.INCH) + 0.1 > 12 || distSensor.getDistance(DistanceUnit.INCH) - 0.1 < 12) { //change this later
                PIDDrive.setTargetState(12 - distSensor.getDistance(DistanceUnit.INCH), odometry.getYCoordinate(), 0);
                overriding = true;
            }
            //above is the exact same thing but when on backstage
            if (numberOfPointsReached < targetStates.length) { //movement
                if (overriding == false) {
                    PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1], targetStates[numberOfPointsReached][2]);
                }



            }
        }
    }
}