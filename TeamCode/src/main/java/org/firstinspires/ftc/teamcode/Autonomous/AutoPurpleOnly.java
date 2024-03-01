package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class AutoPurpleOnly extends AutoDeadReckoning {
    // parking to right
    int parkDirection = 1;
    @Override
    public void drive(){
        //parking direction, negative is to the left and positive is to the right
        int parkDirection = 1;
        if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(14d, 0.8);
            deadReckoningDrive.setTargetRotation(60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.8, 5, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 9d, 0.8);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(-10, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            deadReckoningDrive.moveForwardDistance(30, 0.8);

        } else if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8);
            deadReckoningDrive.setTargetRotation(-36, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 9d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 2d, 0.8);
            deadReckoningDrive.setTargetRotation(-93, 0.8);
            deadReckoningDrive.moveRightDistance(13);
            deadReckoningDrive.moveForwardDistance(30, 0.8, 5,true);
        } else {
            deadReckoningDrive.moveForwardDistance(20, 0.8);
            deadReckoningDrive.moveForwardDistance(-18, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(-92, 0.8);
            deadReckoningDrive.moveForwardDistance(30, 0.8, 5, true);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
