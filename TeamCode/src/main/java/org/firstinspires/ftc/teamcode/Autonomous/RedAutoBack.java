package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class RedAutoBack extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(14d, 0.8);
            deadReckoningDrive.setTargetRotation(60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 6d, 0.5);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 7d, 0.5);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8);
            deadReckoningDrive.setTargetRotation(-38, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 9d, 0.5);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 3d, 0.5);
        } else {
            deadReckoningDrive.moveForwardDistance(20, 0.6);
            deadReckoningDrive.moveForwardDistance(-2d, 0.6);
        }

        // go to backdrop
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            deadReckoningDrive.moveForwardDistance(22, 0.8);
            deadReckoningDrive.moveRightDistance(-5);
            deadReckoningDrive.setTargetRotation(-90, 0.8);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            deadReckoningDrive.moveForwardDistance(23, 0.8);
        } else {
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            deadReckoningDrive.moveForwardDistance(25, 0.8);
        }

        // Aligning code, may be broken from here on
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveRightDistance(-15d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.4, 0.4, 0.4, 0.4);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
//            boxServo.setPosition(boxServoLeft);
            while(!boxServoController.setServoPosition(boxServoLeft, 1.5, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2, 0.5);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-16);
            deadReckoningDrive.moveForwardDistance(2);

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setMotorPowersForTime(2d, 0.4, 0.4, 0.4, 0.4);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
//            boxServo.setPosition(boxServoRight);
            while(!boxServoController.setServoPosition(boxServoRight, 1.5, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2, 0.5);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-24);
            deadReckoningDrive.moveForwardDistance(2);

        } else {
            deadReckoningDrive.moveRightDistance(-1d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.4, 0.4, 0.4, 0.4);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 1.5, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2, 0.5);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-22);
            deadReckoningDrive.moveForwardDistance(2);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
