package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoBack extends Auto {
    final int positionNumber = 2;
    double [][] targetStates = {{84, 132, 0},
            {120, 108, 0},
            {84, 132, 0},
            {12, 132, 0},
            {12, 68, 0},
            {36, 132, 0},
            {84, 132, 0}
    };

    double [][] teamPropPos = {
            {0,0,0}
    };
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
        return 4;
    }

    @Override
    public int getPositionNumber(){
        return positionNumber;//Position Number is 4
    }

    @Override
    public CameraEnums.CameraModes getColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}
