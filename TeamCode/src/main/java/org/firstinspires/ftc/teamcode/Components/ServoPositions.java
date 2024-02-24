package org.firstinspires.ftc.teamcode.Components;

public interface ServoPositions {
    double wristServoIn = 0.223;
    double wristServoOut = 0.4725;
    double wristServoFlat = 0;

    double boxServoNeutral = 0.5115;
    double boxServoRight = 0.5115 - 0.72 + 0.5115;
    double boxServoLeft = 0.72;

    double launcherOpen = 0.5;
    double launcherClosed = 0.6;


    double intakeIdle = 0.19;
    double intakeLowest = 0.20;
    double intakeStackOfTwo = 0.1535;
    double intakeStackOfThree = 0.1185;
    double intakeStackOfFour = 0.1315;
    double intakeStackOfFive = 0.076;
    double intakeHighest = 0.066;
    
    double elbowServoOut = 0.719;
    double elbowServoVertical = 0.1865;
    double elbowServoIn = 0.0455;
}
