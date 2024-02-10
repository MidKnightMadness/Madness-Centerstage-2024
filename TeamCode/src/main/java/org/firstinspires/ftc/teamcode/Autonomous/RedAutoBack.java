package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class RedAutoBack extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(12d, 0.8);
            deadReckoningDrive.setTargetRotation(60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.5, 5, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 6d, 0.5);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8);
            deadReckoningDrive.setTargetRotation(-38, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 7d, 0.5);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 3d, 0.5);
        } else {
            deadReckoningDrive.moveForwardDistance(19, 0.6);
            deadReckoningDrive.moveForwardDistance(-3d, 0.6);
        }

        // go to backdrop
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(-94, 0.8);
            deadReckoningDrive.moveForwardDistance(20, 0.8, 5, true);
            deadReckoningDrive.moveRightDistance(-4);
            deadReckoningDrive.setTargetRotation(-94, 0.8);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.setTargetRotation(-94, 0.8);
            deadReckoningDrive.moveForwardDistance(23, 0.8, 5, true);
        } else {
            deadReckoningDrive.setTargetRotation(-94, 0.8);
            deadReckoningDrive.moveForwardDistance(25, 0.8, 5, true);
        }

        // Aligning code, may be broken from here on
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveRightDistance(-14d);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoLeft, 1.5, telemetry)){}
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2, 0.5);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(16);
            stop();

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 1.5, telemetry)){}
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2, 0.5);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(8);
            stop();

        } else {
            deadReckoningDrive.moveRightDistance(-1);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 1.5, telemetry)){}
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2, 0.5);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(10);
            stop();
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
