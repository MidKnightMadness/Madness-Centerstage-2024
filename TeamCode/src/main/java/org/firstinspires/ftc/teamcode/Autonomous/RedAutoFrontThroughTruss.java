package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class RedAutoFrontThroughTruss extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(2d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(35, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 10d, 0.8, 3, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8, 3, true);
            deadReckoningDrive.moveForwardDistance(-14d, 0.5, 1, true);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-94, 0.6);
            deadReckoningDrive.moveForwardDistance(35, 0.6, 0);
            sleep(500);
            deadReckoningDrive.setTargetRotation(-60, 0.6);
            deadReckoningDrive.moveForwardDistance(25, 0.6, -93, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(-90, 0.6);
            deadReckoningDrive.moveRightDistance(-8);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 0.75, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);
//
//            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2);
//            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);
//
//            // Park
            deadReckoningDrive.moveForwardDistance(-2, 1.0);
            deadReckoningDrive.moveRightDistance(-21);
            deadReckoningDrive.moveForwardDistance(6);

        } else if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(14d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(-60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.8, 3, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(-14d, 0.8, 1, true);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-93, 0.6); // Set power level for rotation adjustment on all autons to thos
            deadReckoningDrive.moveForwardDistance(35, 0.6, -93);
            sleep(500);
            deadReckoningDrive.setTargetRotation(-60, 0.25); // Set power level for rotation adjustment on all autons to thos
            deadReckoningDrive.moveForwardDistance(25, 0.6, -90, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 0.75, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);
//
//            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2);
//            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);
//
//            // Park
            deadReckoningDrive.moveForwardDistance(-2, 1.0);
            deadReckoningDrive.moveRightDistance(-16d);
            deadReckoningDrive.moveForwardDistance(6);

        } else {
            deadReckoningDrive.moveForwardDistance(20, 0.6, 3, true);
            deadReckoningDrive.moveForwardDistance(-2, 0.6, 3, true);

            // Go through truss
            deadReckoningDrive.setTargetRotation(-93, 0.4);
            deadReckoningDrive.moveForwardDistance(30, 0.8, -93);
            sleep(2000);
            deadReckoningDrive.setTargetRotation(-93, 0.3);
            deadReckoningDrive.moveForwardDistance(35, 0.8, -93, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(-90, 0.7);
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
//
//            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2);
//            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);
//
//            // Park
            deadReckoningDrive.moveForwardDistance(-2, 1.0);
            deadReckoningDrive.moveRightDistance(-24);
            deadReckoningDrive.moveForwardDistance(6);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}

