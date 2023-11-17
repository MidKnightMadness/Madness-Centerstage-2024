package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoBack extends Auto {
    double[][] targetStates = {
            {0, 0, 0}, //insert starting coords
        {84, 12, 0},
    {120, 36, 0},
    {84, 12, 0},
    {12, 12, 0},
    {12, 36, 0},
    {36, 12, 0},
    {84, 12, 0}

    };
    @Override
    public double[][] setTargetStates(){
        return targetStates;
    }
    final int positionNumber = 2;
    @Override
    public int getDirection() {
        return 1;
    }

    @Override
    public int getNumTilesToPark() {
        return 4;
    }

    @Override
    public int getPositionNumber(){
        return positionNumber;//Position Number is 2
    }
}
