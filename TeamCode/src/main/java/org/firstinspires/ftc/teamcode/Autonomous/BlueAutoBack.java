package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoBack extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(18d);
            deadReckoningDrive.setTargetRotation(-60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5.25d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5.25d);
            deadReckoningDrive.setTargetRotation(90);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(5d);
            deadReckoningDrive.setTargetRotation(33);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 4d);
            deadReckoningDrive.setTargetRotation(90);
        } else {
            deadReckoningDrive.moveForwardDistance(27d);
            deadReckoningDrive.moveForwardDistance(-9);
        }

        // go to backdrop
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-10d);
            deadReckoningDrive.moveForwardDistance(25d);
            deadReckoningDrive.moveRightDistance(13);
            deadReckoningDrive.setTargetRotation(90);
        } else {
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(27);
        }

        // Aligning code, may be broken from here on
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(7d);
            deadReckoningDrive.setMotorPowersForTime(3, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.375, 1.05);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
//            boxServo.setPosition(boxServoRight); // right
            rotateBoxTo(boxServoRight); // slow
            deadReckoningDrive.moveRightDistance(-18);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.setTargetRotation(95);
            deadReckoningDrive.setMotorPowersForTime(3, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.375, 1.05);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
//            boxServo.setPosition(boxServoLeft); // left
            rotateBoxTo(boxServoLeft); // slow
            deadReckoningDrive.moveRightDistance(-10);

        } else {
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(3d);
            deadReckoningDrive.setMotorPowersForTime(3, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.375, 1.05);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
//            boxServo.setPosition(boxServoRight); // right
            rotateBoxTo(boxServoRight); // slow
            deadReckoningDrive.moveRightDistance(-15);
        }

    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}
