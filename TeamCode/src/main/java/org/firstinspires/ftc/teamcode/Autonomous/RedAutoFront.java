package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;


@Autonomous
public class RedAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(18d, 0.8);
            deadReckoningDrive.setTargetRotation(-60, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 6d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 6d, 0.8);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(27d, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(70, 0.5, 0);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(22);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, 2);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoRight); // right
//            rotateBoxTo(boxServoRight); // slow

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(2d, 0.8);
            deadReckoningDrive.setTargetRotation(33, 0.8);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 15d, 0.8);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d, 0.8);
            deadReckoningDrive.setTargetRotation(-45, 0.8);
            deadReckoningDrive.moveForwardDistance(8, 0.8);
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveForwardDistance(25, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(70, 0.5, 0);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.moveRightDistance(14);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, 2);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoLeft); // left
//            rotateBoxTo(boxServoLeft); // slow

        } else {
            deadReckoningDrive.moveForwardDistance(23, 0.8);
            deadReckoningDrive.moveForwardDistance(-3, 0.8);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-90, 0.8);
            imu.resetYaw();
            deadReckoningDrive.moveForwardDistance(70, 0.5, 0);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(0, 0.8);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.5, 2);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            while(!boxServoController.setServoPosition(boxServoRight, 1, telemetry)){}
//            rotateBoxTo(boxServoLeft); // slow
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
