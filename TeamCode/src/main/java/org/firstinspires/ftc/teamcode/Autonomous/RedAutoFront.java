package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;


@Autonomous
public class RedAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(14d, 0.8);
            deadReckoningDrive.setTargetRotation(-60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 3.5d, 0.8, 2, true);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 3.5d, 0.8);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(22d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-94, 0.8);
            sleep(250);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(28, 0.8, 0);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(30, 0.8, 0, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(14);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 2, telemetry)){}
            sleep(750);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-24);
            stop();

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(2d, 0.5);
            deadReckoningDrive.setTargetRotation(30, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 12d, 0.6);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.6);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(4);
            deadReckoningDrive.moveForwardDistance(20, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-92, 0.5);
            sleep(250);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(28, 0.8, 0);
            deadReckoningDrive.setTargetRotation(0);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(30, 0.8, 0, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(5);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoLeft, 2, telemetry)){}
            sleep(750);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-16);
            stop();

        } else {
            deadReckoningDrive.moveForwardDistance(18.5, 0.8);
            deadReckoningDrive.moveForwardDistance(-2.5, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-94, 0.6);
            sleep(250);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(28, 0.8, 0);
            deadReckoningDrive.setTargetRotation(0);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(30, 0.8, 0, 5);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(-1);
            deadReckoningDrive.setMotorPowersForTime(2d, rammingPower, rammingPower, rammingPower, rammingPower);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 2, telemetry)){}
            sleep(750);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            sleep(500);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-22);
            stop();
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
