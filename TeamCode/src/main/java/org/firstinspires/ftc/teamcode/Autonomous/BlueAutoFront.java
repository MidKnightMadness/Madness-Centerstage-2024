package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(15d, 0.8);
            deadReckoningDrive.setTargetRotation(60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 3.5d, 0.6);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 3.5d, 0.6);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(22d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(92, 0.5);
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
            deadReckoningDrive.moveForwardDistance(2d, 0.8);
            deadReckoningDrive.setTargetRotation(-30, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 12d, 0.6);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.6);
            deadReckoningDrive.setTargetRotation(45, 0.8);
            deadReckoningDrive.moveForwardDistance(6d, 0.6);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(25, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(92, 0.4);
            sleep(250);
            deadReckoningDrive.moveForwardDistance(63, 0.8, 90);

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
            deadReckoningDrive.moveForwardDistance(21.5, 0.8);
            deadReckoningDrive.moveForwardDistance(-1, 0.8);

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

