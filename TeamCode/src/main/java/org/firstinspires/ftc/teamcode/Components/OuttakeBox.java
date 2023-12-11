package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OuttakeBox { // Includes wrist and elbow motors
    // Hardware
    Servo boxServo;
    Servo leftWrist; // Only using right side rn
    Servo rightWrist;
    Servo leftElbow;
    Servo rightElbow;

    // Servo edge bounds
    // Box
    double [] boxBounds = {0.8615, 0.578}; // right, left
    double boxServoDefault = (boxBounds [0] + boxBounds [1]) / 2.0d;
    // Right Side
    double [] rightWristBounds = {0.25, 1.0d}; // outboard, inboard
    // Left Side
    double [] leftWristBounds = {0.85, 0.1}; // outboard, inboard

    // Outtake elbow servos
    // Right Side
    double [] rightElbowBounds = {0.85, 0.1}; // outboard, inboard
    // Left Side
    double [] leftElbowBounds = {0.85, 0.1}; // outboard, inboard

    //
    double [] extensionLengthBounds; // Currently unused due to encoder issues
    double [] rightElbowWaypoints = {0.0, 0.0, 0.0, 0.0}; // Start, mid-stage, up, release
    double [] rightWristWaypoints = {0.0, 0.0, 0.0, 0.0}; // Start, mid-stage, up, release
    double [] leftElbowWaypoints = {0.0, 0.0, 0.0}; // Currently not used
    double [] leftWristWaypoints = {0.0, 0.0, 0.0}; // Currently not used

    // Other variables
    ElapsedTime timer;
    int outtakeTime = 0; // Milliseconds

    public OuttakeBox(HardwareMap hardwareMap, String tag){
        boxServo = hardwareMap.get(Servo.class, tag);
        rightWrist = hardwareMap.get(Servo.class, "Right wrist servo");
        rightElbow = hardwareMap.get(Servo.class, "Right elbow servo");

        timer = new ElapsedTime();
    }

    public void outtakeRight(){
        timer.reset();
        while(timer.milliseconds() < outtakeTime){
            boxServo.setPosition(boxBounds [0]);
        }
        boxServo.setPosition(boxServoDefault);
    }

    public void outtakeLeft(){
        timer.reset();
        while(timer.milliseconds() < outtakeTime){
            boxServo.setPosition(boxBounds [1]);
        }
        boxServo.setPosition(boxServoDefault);
    }

    public void setExtensionStage(int stage){
        rightWrist.setPosition(rightWristWaypoints [stage]);
        leftWrist.setPosition(leftWristWaypoints [stage]);
    }
}
