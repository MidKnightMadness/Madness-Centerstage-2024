package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;


@Autonomous
public class RedAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(16d, 0.8);
            deadReckoningDrive.setTargetRotation(-60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 5d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(22d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-92, 0.3);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(67, 0.5, 0);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(13);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 2, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-24);
            deadReckoningDrive.moveForwardDistance(2);

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(2d, 0.8);
            deadReckoningDrive.setTargetRotation(33, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8);
            deadReckoningDrive.setTargetRotation(-45, 0.8);
            deadReckoningDrive.moveForwardDistance(8, 0.8);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(22, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-92, 0.3);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(67, 0.5, 0);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(14);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoLeft, 2, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.setMotorPowersForTime(1d, rammingPower, rammingPower, rammingPower, rammingPower);

            // Park
            deadReckoningDrive.moveForwardDistance(-2);
            deadReckoningDrive.moveRightDistance(-16);
            deadReckoningDrive.moveForwardDistance(2);

        } else {
            deadReckoningDrive.moveForwardDistance(23, 0.8);
            deadReckoningDrive.moveForwardDistance(-3, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-92, 0.3);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(70, 0.5, 0);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(-0.5);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, slidesExtensionTimeConstant);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            while(!boxServoController.setServoPosition(boxServoRight, 2, telemetry)){}
            sleep(500);
            rightWristServo.setPosition(wristServoIn);
            boxServo.setPosition(boxServoNeutral);
            slides.extendForTime(-0.5, -0.5, slidesExtensionTimeConstant);

            // Hit backdrop to make sure pixel is scored
            deadReckoningDrive.moveForwardDistance(-2);
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
