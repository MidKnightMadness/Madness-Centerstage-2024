package org.firstinspires.ftc.teamcode.Components;

public interface ServoPositions {
    double wristServoIn = 0.226;
    double wristServoOut = 0.4725;
    double wristServoFlat = 0;

    double boxServoNeutral = 0.5115;
    double boxServoRight = 0.768;
    double boxServoLeft = 0.5115 - 0.768 - 0.5115;

    double launcherOpen = 0.5;
    double launcherClosed = 0.6;


    double intakeDefault = 0.12;
    double intakeLowest = 0.19;
    double intakeHighest = 0.066;

}
