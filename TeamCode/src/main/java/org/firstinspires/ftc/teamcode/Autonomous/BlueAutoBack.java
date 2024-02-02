package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoBack extends AutoDeadReckoning {
    @Override
    public void drive() {
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(15d, 0.8, 0);
            deadReckoningDrive.setTargetRotation(-60, 0.7);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5.25d, 0.8, -60);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5.25d, 0.8, -60);
            deadReckoningDrive.setTargetRotation(90, 0.7);
        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8, 0);
            deadReckoningDrive.setTargetRotation(33, 0.7);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d, 0.8, 33);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 4d);
            deadReckoningDrive.setTargetRotation(90, 0.7);
        } else {
            deadReckoningDrive.moveForwardDistance(25d, 0.8, 0);
            deadReckoningDrive.moveForwardDistance(-7, 0.8, 0);
        }

        // go to backdrop
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(25d);
            deadReckoningDrive.setTargetRotation(90);
        } else {
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(27);
        }

        // Aligning code, may be broken from here on
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(5d);
            deadReckoningDrive.setMotorPowersForTime(3, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoRight); // right

            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            sleep(500);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-33);
            deadReckoningDrive.moveForwardDistance(6);


        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.setTargetRotation(95);
            deadReckoningDrive.setMotorPowersForTime(3, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoLeft); // left

            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            sleep(500);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-28);
            deadReckoningDrive.moveForwardDistance(6);

        } else {
            deadReckoningDrive.setTargetRotation(90, 0.8);
            deadReckoningDrive.moveRightDistance(1);
            deadReckoningDrive.setMotorPowersForTime(3, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            while(!boxServoController.setServoPosition(boxServoRight, 1.25, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            sleep(500);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-23);
            deadReckoningDrive.moveForwardDistance(6);
        }

    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}
