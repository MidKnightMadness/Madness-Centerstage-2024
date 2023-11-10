package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;

import org.firstinspires.ftc.teamcode.Intake.Intake;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import com.qualcomm.robotcore.hardware.DistanceSensor;


@TeleOp(name = "AutonomousPixels")
public class Autonomous extends OpMode {

    /*

Cycle

Opmode 1 and 2 are team color red -> teamColor = 1
Opcode 3 and 4 are team color blue -> teamColor = 2

     */

    double[][] targetStates = {
            //add additional coords for getting 'close enough' for intake and outtake
            //red
            //5 total points: Outtake, rightmost (or leftmost), down past trusses, intake stacks
            //front red also has {36, 12}
            {84, 12, 0},
            {120, 36, 0},
            {84, 12, 0},
            {24, 12, 0},
            {12, 36, 0},
            {24, 12, 0},
            {84, 12, 0},
            //and repeat
            {120, 36, 0}, {84, 12, 0}, {24, 12, 0}, {12, 36, 0}, {24, 12, 0}, {84, 12, 0}, {120, 36, 0}, {84, 12, 0}, {24, 12, 0}, {12, 36, 0}, {24, 12, 0}, {84, 12, 0},
            //blue starting here
            //front blue also has {36, 132}
            {84, 132, 0},
            {120, 108, 0},
            {84, 132, 0},
            {24, 132, 0},
            {12, 108, 0},
            {24, 132, 0},
            {84, 132, 0},
            //and repeat
           {120, 108, 0}, {84, 132, 0}, {24, 132, 0}, {12, 108, 0}, {24, 132, 0}, {84, 132, 0},  {120, 108, 0}, {84, 132, 0}, {24, 132, 0}, {12, 108, 0}, {24, 132, 0}, {84, 132, 0}


    };
    DistanceSensor distSensor;

    int teamColor = 0; //put this somewhere else; red is 3, blue is 1
    int numberOfPointsReached = 0;
    /*if (teamColor ==3)

    {
        numberOfPointsReached += a lot;
    }*/
    MecanumDrive drive;
    Odometry odometry;
    PIDDrive PIDDrive;
    DcMotorEx IntakeMotor;
    Servo servoBox;
    Servo armIntake;
    Servo ManipulatorServoLeft;
            Servo ManiuplatorServoRight;
    ElapsedTime timer;
    boolean taskComplete = false; //just to make a few things run for now
    boolean teamPropTask = false;
    int nextTask = 2; //1 is for intake, and 2 is for outtake

    int stored = 1; //the starting pixel in robot

    @Override
    public void init() {
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "servoIntake");
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));
        PIDDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);
        timer = new ElapsedTime();
        timer.startTime();
        distSensor = hardwareMap.get(DistanceSensor.class, "distSensor");
        //implement color or touch sensor for intake/storage box
        //put in camera for localization
        servoBox = hardwareMap.get(Servo.class, "servoBox");
        armIntake = hardwareMap.get(Servo.class,"armIntakeServo");
        ManiuplatorServoRight = hardwareMap.get(Servo.class,"ManipulatorServoRight");
        ManipulatorServoLeft = hardwareMap.get(Servo.class,"ManipulatorServoLeft");

        PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1], targetStates[numberOfPointsReached][2]);
    }

    @Override
    public void loop() {

        //while the task at location is complete
        odometry.updatePosition();
        PIDDrive.updatePID();
        if (timer.time() < 23) {
            if (teamPropTask == false) {
                //set 3 if statements and leniency for each one
                PIDDrive.setTargetState(teamProp.x, teamProp.y, teamColor * Math.PI / 2);
                if (PIDDrive.distanceToTarget < 0.1) {
                    teamPropTask = true;
                    IntakeMotor.setPower(-1);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    IntakeMotor.setPower(0);
                }

            } else {
                if (targetStates[numberOfPointsReached-1][0] == 12 || targetStates[numberOfPointsReached-1][0] == 120) { //arbitrary numbers check if it's one of the points where you run intake or outtake
                    taskComplete = false;
                }

                if (taskComplete == false) { //for both intake and outtake
                    if (nextTask == 1) {//if currently we're in stacks
                        //intake
                        //stored++ for each intake
                        while (stored != 2){
                            IntakeMotor.setPower(1);
                            armIntake.setPosition(armIntake.getPosition()-0.1);
                            //lower at rate
                            if (pixel Passes){
                                stored++;
                            }
                        }
                        taskComplete = true;
                        nextTask = 2;
                        armIntake.setPosition(1);
                    }
                } else {
                    //outtake
                    //stored-- each time
                    if (stored != 0){
                        if (ManipulatorServoLeft.getPosition() != 0.5){

                                    ManiuplatorServoRight.setPosition(0.5);
                            ManipulatorServoLeft.setPosition(0.5);
                        }
                        else{
                            servoBox.setPosition(1);
                        }
                    }
                    else{
                        taskComplete = true;
                        nextTask = 1;
                        servoBox.setPosition(0);
                    }
                }
                //help how do i


                if (taskComplete == true) {
                    if (PIDDrive.distanceToTarget < 0.1) { //when we reach the desired point, then our next point is queued
                        numberOfPointsReached++;
                    }
                    //above is the exact same thing but when on backstage
                    if (numberOfPointsReached < targetStates.length) { //movement
                            PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1], targetStates[numberOfPointsReached][2]);
                    }
                }
            }

        }else{
            if (teamColor == 3){
                PIDDrive.setTargetState(odometry.getXCoordinate(), 12, 0);
                if (PIDDrive.distanceToTarget < 0.1){
                    PIDDrive.setTargetState(130, 12, 0);
                }
            }
            else{
                PIDDrive.setTargetState(odometry.getXCoordinate(), 132, 0);
                if (PIDDrive.distanceToTarget < 0.1){
                    PIDDrive.setTargetState(130, 132, 0);
                }

            }

        }
    }
    }
