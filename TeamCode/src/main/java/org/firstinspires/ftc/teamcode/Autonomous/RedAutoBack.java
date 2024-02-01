package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class RedAutoBack extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(16d);
            deadReckoningDrive.setTargetRotation(60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 8d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 8d);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d);
            deadReckoningDrive.setTargetRotation(-42);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 4d);
        } else {
            deadReckoningDrive.moveForwardDistance(25d);
            deadReckoningDrive.moveForwardDistance(-7d);
        }

        // go to backdrop
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(-92);
            deadReckoningDrive.moveForwardDistance(22);
            deadReckoningDrive.moveRightDistance(-5);
            deadReckoningDrive.setTargetRotation(-90);
        } else {
            deadReckoningDrive.setTargetRotation(-92);
            deadReckoningDrive.moveForwardDistance(27);
        }

        // Aligning code, may be broken from here on
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveRightDistance(-10d);
            deadReckoningDrive.setMotorPowersForTime(3d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoLeft); // right
//            rotateBoxTo(boxServoRight);
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.7, 0.7, 0.7, 0.7);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(17);
            deadReckoningDrive.moveForwardDistance(6);

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setMotorPowersForTime(3d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoRight); // left
//            rotateBoxTo(boxServoRight);
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.7, 0.7, 0.7, 0.7);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(8);
            deadReckoningDrive.moveForwardDistance(6);

        } else {
            deadReckoningDrive.moveRightDistance(-6d);
            deadReckoningDrive.setMotorPowersForTime(3d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoRight); // right
//            rotateBoxTo(boxServoRight);
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.7, 0.7, 0.7, 0.7);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(12);
            deadReckoningDrive.moveForwardDistance(6);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
