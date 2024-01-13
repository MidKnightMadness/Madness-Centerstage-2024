package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class RedAutoBack extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(18d);
            deadReckoningDrive.setTargetRotation(60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 8d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 8d);
            deadReckoningDrive.setTargetRotation(-90);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d);
            deadReckoningDrive.setTargetRotation(-42);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 4d);
            deadReckoningDrive.setTargetRotation(-90);
        } else {
            deadReckoningDrive.moveForwardDistance(27d);
            deadReckoningDrive.moveForwardDistance(-9);
        }

        // go to backdrop
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveRightDistance(10d);
            deadReckoningDrive.moveForwardDistance(25d);
            deadReckoningDrive.moveRightDistance(-13);
            deadReckoningDrive.setTargetRotation(-90);
        } else {
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveForwardDistance(27);
        }

        // Aligning code, may be broken from here on
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveRightDistance(-10d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            double extensionStartingTime = timer.getTime();
//            while(timer.getTime() < extensionStartingTime + 2){
//                motorLeft.setPower(0.5);
//                motorRight.setPower(0.5);
//            }
//            sleep(500);
//            rightWristServo.setPosition(wristVertical);
//            sleep(250);
//            boxServo.setPosition(0.45); // right
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveRightDistance(-1d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            double extensionStartingTime = timer.getTime();
//            while(timer.getTime() < extensionStartingTime + 2){
//                motorLeft.setPower(0.5);
//                motorRight.setPower(0.5);
//            }
//            sleep(500);
//            rightWristServo.setPosition(wristVertical);
//            sleep(250);
//            boxServo.setPosition(0.85); // left
        } else {
            deadReckoningDrive.moveRightDistance(-5d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
