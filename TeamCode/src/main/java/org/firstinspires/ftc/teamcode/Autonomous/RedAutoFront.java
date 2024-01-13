package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;


@Autonomous
public class RedAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(18d);
            deadReckoningDrive.setTargetRotation(-60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 6d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 6d);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(27d);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveForwardDistance(75);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveRightDistance(22);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.375, 1.05);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoRight); // right

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(2d);
            deadReckoningDrive.setTargetRotation(33);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 15d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 5d);
            deadReckoningDrive.setTargetRotation(-45);
            deadReckoningDrive.moveForwardDistance(8);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(25);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveForwardDistance(75);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveRightDistance(14);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.375, 1.05);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoLeft); // left

        } else {
            deadReckoningDrive.moveForwardDistance(27);
            deadReckoningDrive.moveForwardDistance(-3);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.moveForwardDistance(75);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(-90);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
            slides.extendForTime(0.5, 0.375, 1.05);
            sleep(500);
            rightWristServo.setPosition(wristServoOut);
            sleep(750);
            boxServo.setPosition(boxServoLeft); // left
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
}
