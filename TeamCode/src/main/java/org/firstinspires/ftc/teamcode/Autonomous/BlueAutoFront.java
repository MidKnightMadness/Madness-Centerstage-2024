package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(15d, 0.8);
            deadReckoningDrive.setTargetRotation(60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 8d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 8d, 0.8);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(28d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(75, 0.6);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-28);
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(-0.5, -0.5, 2);
//            sleep(500);
            rightWristServo.setPosition(wristServoOut);
//            sleep(750);
            boxServo.setPosition(boxServoLeft); // left
//            rotateBoxTo(boxServoLeft); // slow

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8);
            deadReckoningDrive.setTargetRotation(-40);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 10d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 10d, 0.8);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(39d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(75, 0.6);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-17);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(-0.5, -0.5, 2);
//            sleep(500);
            rightWristServo.setPosition(wristServoOut);
//            sleep(750);
            boxServo.setPosition(boxServoRight); // right
//            rotateBoxTo(boxServoRight); // slow

        } else {
            deadReckoningDrive.moveForwardDistance(27, 0.8);
            deadReckoningDrive.moveForwardDistance(-1.5, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(75, 0.6);

            // Align to backdrop
            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(1);
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(-0.5, -0.5, 2);
//            sleep(500);
            rightWristServo.setPosition(wristServoOut);
//            sleep(750);
            boxServo.setPosition(boxServoRight); // right
//            rotateBoxTo(boxServoRight); // slow
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}

