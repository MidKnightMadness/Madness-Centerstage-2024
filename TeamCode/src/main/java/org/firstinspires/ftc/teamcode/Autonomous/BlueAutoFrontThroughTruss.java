






package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;

@Autonomous
public class BlueAutoFrontThroughTruss extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(13d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(57, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.8, 5, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(-12d, 0.8, 3, true);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90, 0.6);
            deadReckoningDrive.moveForwardDistance(35, 0.7, 88);
            sleep(500);
            deadReckoningDrive.setTargetRotation(50, 0.4);
            deadReckoningDrive.moveForwardDistance(25, 0.8, 60, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90, 0.8);
            deadReckoningDrive.moveRightDistance(-4d);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
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
//            deadReckoningDrive.moveForwardDistance(-2);
//            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-3, 1.0);
            deadReckoningDrive.moveRightDistance(28);
//            deadReckoningDrive.moveForwardDistance(6);

        } else if (getSpikeMarkPosition() == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(2d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(-36, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 10.5d, 0.8, 3, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8, 3, true);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(-10d, 0.8, 1, true);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(91.5, 0.5); // Set power level for rotation adjustment on all autons to thos
            deadReckoningDrive.moveForwardDistance(39, 0.6, 0);
            sleep(500);
            deadReckoningDrive.setTargetRotation(40, 0.5); // Set power level for rotation adjustment on all autons to thos
            deadReckoningDrive.moveForwardDistance(25, 0.6, 60, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(1);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
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
//            deadReckoningDrive.moveForwardDistance(-2);
//            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-3, 1.0);
            deadReckoningDrive.moveRightDistance(14d);
//            deadReckoningDrive.moveForwardDistance(6);

        } else {
            deadReckoningDrive.moveForwardDistance(20, 0.8, 3, true);
            deadReckoningDrive.moveForwardDistance(-1.5, 0.8, 3, true);

            deadReckoningDrive.setTargetRotation(93, 0.5);

            // Go through stage door
            deadReckoningDrive.moveForwardDistance(30, 0.6, 90);
            sleep(10000);
            deadReckoningDrive.setTargetRotation(93, 0.4);
            deadReckoningDrive.moveForwardDistance(35, 0.6, 60, 5);

            // Align to backdrop
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 1.5, telemetry)){}
            sleep(750);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
//            deadReckoningDrive.moveForwardDistance(-2);
//            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-3, 1.0);
            deadReckoningDrive.moveRightDistance(20);
//            deadReckoningDrive.moveForwardDistance(6);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}