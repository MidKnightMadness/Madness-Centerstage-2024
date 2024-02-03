package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(15d, 0.8);
            deadReckoningDrive.setTargetRotation(60, 0.7);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 6d, 0.6);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 6d, 0.6);
            deadReckoningDrive.setTargetRotation(0, 0.7);
            deadReckoningDrive.moveForwardDistance(26d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90, 0.4);
            deadReckoningDrive.moveForwardDistance(65, 0.5, 90);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90, 0.7);
//            deadReckoningDrive.moveRightDistance(-29, 0.7);
//            deadReckoningDrive.setMotorPowersForTime(2d, 0.4, 0.4, 0.4, 0.4);
            deadReckoningDrive.strafeUntilBackdrop(rangeSensor, false);
            deadReckoningDrive.moveRightDistance(-13d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoLeft, 1.25, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
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

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d, 0.8);
            deadReckoningDrive.setTargetRotation(-40);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 9d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 9d, 0.8);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(32d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90, 0.4);
            deadReckoningDrive.moveForwardDistance(65, 0.5, 90);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
//            deadReckoningDrive.moveRightDistance(-26);
//            deadReckoningDrive.setMotorPowersForTime(2d, 0.4, 0.4, 0.4, 0.4);
            deadReckoningDrive.strafeUntilBackdrop(rangeSensor, false);
            deadReckoningDrive.moveRightDistance(-18d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 1.25, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(13d);
            deadReckoningDrive.moveForwardDistance(6);

        } else {
            deadReckoningDrive.moveForwardDistance(25d, 0.6, 0);
            deadReckoningDrive.moveForwardDistance(-1, 0.6, 0);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90, 0.4);
            deadReckoningDrive.moveForwardDistance(65, 0.5, 90);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90, 0.7);
//            deadReckoningDrive.setTargetRotation(90, 0.7);
//            deadReckoningDrive.setMotorPowersForTime(2d, 0.4, 0.4, 0.4, 0.4);
            deadReckoningDrive.strafeUntilBackdrop(rangeSensor, false);
            deadReckoningDrive.moveRightDistance(-8d);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.3, 0.3, 0.3, 0.3);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 1.25, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(11);
            deadReckoningDrive.moveForwardDistance(6);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}

