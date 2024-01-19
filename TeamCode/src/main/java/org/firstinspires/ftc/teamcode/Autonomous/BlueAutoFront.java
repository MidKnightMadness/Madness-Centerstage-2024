package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(18d);
            deadReckoningDrive.setTargetRotation(60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 8d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 8d);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(25d);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(100);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-25);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            slides.extendForTime(0.5, 0.375, 1.05);
//            sleep(500);
//            rightWristServo.setPosition(wristServoOut);
//            sleep(750);
//            boxServo.setPosition(boxServoLeft); // left
//            rotateBoxTo(boxServoLeft); // slow

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d);
            deadReckoningDrive.setTargetRotation(-42);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 11d);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(40d);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(100);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-16);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            slides.extendForTime(0.5, 0.375, 1.05);
//            sleep(500);
//            rightWristServo.setPosition(wristServoOut);
//            sleep(750);
//            boxServo.setPosition(boxServoRight); // right
//            rotateBoxTo(boxServoRight); // slow

        } else {
            deadReckoningDrive.moveForwardDistance(27);
            deadReckoningDrive.moveForwardDistance(-1);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(75);

            // Align to backdrop
            // Align to backdrop
            deadReckoningDrive.setTargetRotation(100);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.2, 0.2, 0.2, 0.2);
//            slides.extendForTime(0.5, 0.375, 1.05);
//            sleep(500);
//            rightWristServo.setPosition(wristServoOut);
//            sleep(750);
//            boxServo.setPosition(boxServoRight); // right
//            rotateBoxTo(boxServoRight); // slow
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}

