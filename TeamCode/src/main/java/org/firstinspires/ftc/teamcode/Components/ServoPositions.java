package org.firstinspires.ftc.teamcode.Components;

public interface ServoPositions {
    double wristServoIn = 0.226;
    double wristServoOut = 0.4725;
    double wristServoFlat = 0;

    double boxServoNeutral = 0.5115;
    double boxServoRight = 0.5115 - 0.768 - 0.5115;
    double boxServoLeft = 0.768;

    double launcherOpen = 0.5;
    double launcherClosed = 0.6;


    double intakeDefault = 0.19;
    double intakeLowest = 0.20;
    double intakeStackOfThree = 0.1185;
    double intakeStackOfTwo = 0.1535;
    double intakeHighest = 0.066;

}
