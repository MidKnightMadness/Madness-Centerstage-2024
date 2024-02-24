package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoBack extends AutoDeadReckoning {
    @Override
    public void drive() {
        //parking direction, negative is to the left and positive is to the right
        int parkDirection = -1;
        if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(13d, 0.8, 0);
            deadReckoningDrive.setTargetRotation(-60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.8, 5, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 9d, 0.8);
            deadReckoningDrive.setTargetRotation(90, 0.8);
        } else if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8, 0);
            deadReckoningDrive.setTargetRotation(36, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 9d, 0.8, 36);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 2d, 0.8);
            deadReckoningDrive.setTargetRotation(90, 0.8);
        } else {
            deadReckoningDrive.moveForwardDistance(20, 0.8, 0);
            deadReckoningDrive.moveForwardDistance(-4, 0.8, 0);
        }

        // go to backdrop
        if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(23d, 0.8, 5, true);
            deadReckoningDrive.setTargetRotation(90, 0.8);
        } else if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.RIGHT){
            deadReckoningDrive.setTargetRotation(93, 0.8);
            deadReckoningDrive.moveForwardDistance(21, 0.8, 5, true);
        } else {
            deadReckoningDrive.setTargetRotation(90, 0.8);
            deadReckoningDrive.moveForwardDistance(21, 0.8, 5, true);
            deadReckoningDrive.moveRightDistance(6, 90);
        }

        // Aligning code, may be broken from here on
        if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveRightDistance(13d, 90);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 0.75, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2, 0.8);
//            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2, 0.8);
            deadReckoningDrive.moveRightDistance(22*parkDirection, 90);
            deadReckoningDrive.moveForwardDistance(6, 0.8);


        } else if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.setTargetRotation(95, 0.8);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoLeft, 0.75, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2, 0.8);
//            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2, 0.8);
            deadReckoningDrive.moveRightDistance(18*parkDirection);
            deadReckoningDrive.moveForwardDistance(6, 0.8);

        } else {
            deadReckoningDrive.setTargetRotation(90, 0.8);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 0.75, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2, 0.8);
//            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2, 0.8);



            deadReckoningDrive.moveRightDistance(24*parkDirection, 0.83);
            deadReckoningDrive.moveForwardDistance(6, 0.8);
        }

    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}
