package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(13d, 0.8);
            deadReckoningDrive.setTargetRotation(57, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.6, 5, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.6);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(1d);
            deadReckoningDrive.moveForwardDistance(20d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(92, 0.3);
            deadReckoningDrive.moveForwardDistance(25, 0.8, 0);
            sleep(1000);
            deadReckoningDrive.moveForwardDistance(35, 0.8, 90, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(91, 0.3);
            deadReckoningDrive.moveRightDistance(-32d);
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
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(-4);
            deadReckoningDrive.moveForwardDistance(20, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(92, 0.3); // Set power level for rotation adjustment on all autons to thos
            deadReckoningDrive.moveForwardDistance(30, 0.8, 0);
            sleep(500);
            deadReckoningDrive.setTargetRotation(92, 0.25); // Set power level for rotation adjustment on all autons to thos
            deadReckoningDrive.moveForwardDistance(30, 0.8, 90, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-22d);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant + 0.5);
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
            deadReckoningDrive.moveForwardDistance(20, 0.6);
            deadReckoningDrive.moveForwardDistance(-2, 0.6);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90, 0.3);
            deadReckoningDrive.moveForwardDistance(30, 0.8, 0);
            sleep(1000);
            deadReckoningDrive.setTargetRotation(93, 0.3);
            deadReckoningDrive.moveForwardDistance(35, 0.5, 90, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90, 0.7);
            deadReckoningDrive.moveRightDistance(-3);
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
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, 0.5, 0.5, 0.5, 0.5);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(9);
            deadReckoningDrive.moveForwardDistance(6);
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}

