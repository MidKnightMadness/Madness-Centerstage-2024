package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

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

    final int robotPosition = 1;
    @Override
    public double[][] setTargetStates(){
        return targetStates;
    }


    @Override
    public int getDirection() {
        return -1;
    }

    @Override
    public int getNumTilesToPark() {
        return 2;
    }

    @Override
    public CameraEnums.CameraModes getColor(){
        return CameraEnums.CameraModes.BLUE;
    }
    @Override
    public int getPositionNumber(){
        return robotPosition;//Position Number is 3

    }


}

