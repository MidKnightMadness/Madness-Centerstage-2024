package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueAutoFront extends Auto {
    Auto auto;
    double [][] targetStates = {
            {36, 132, 0},
    {84, 132, 0},
    {120, 108, 0},
    {84, 132, 0},
    {12, 132, 0},
    {12, 68, 0},
    {36, 132, 0},
    {84, 132, 0},
    };

    @Override
    public double[][] setTargetStates(){
        return targetStates;
    }

    final int positionNumber = 3;
    @Override
    public int getDirection() {
        return -1;
    }

    @Override
    public int getNumTilesToPark() {
        return 2;
    }

    @Override
    public int getPositionNumber(){
        return 3;//Position Number is 3

    }


}

