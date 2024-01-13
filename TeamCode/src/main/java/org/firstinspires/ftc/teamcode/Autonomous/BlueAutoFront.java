package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;

@Autonomous
public class BlueAutoFront extends AutoDeadReckoning {
    @Override
    public void drive(){
        if (teamPropPosition == CameraEnums.SpikeMarkPositions.LEFT) {
            deadReckoningDrive.moveForwardDistance(18d);
            deadReckoningDrive.setTargetRotation(60);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 8d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 8d);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(25d);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(100);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-25);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            double extensionStartingTime = timer.getTime();
//            while(timer.getTime() < extensionStartingTime + 2){
//                motorLeft.setPower(0.5);
//                motorRight.setPower(0.5);
//            }
//            sleep(500);
//            rightWristServo.setPosition(wristVertical);
//            sleep(250);
//            boxServo.setPosition(0.85); // left

        } else if (teamPropPosition == CameraEnums.SpikeMarkPositions.RIGHT) {
            deadReckoningDrive.moveForwardDistance(5d);
            deadReckoningDrive.setTargetRotation(-42);
            deadReckoningDrive.moveForwardDistance(Math.sqrt(2) * 11d);
            deadReckoningDrive.moveForwardDistance(-Math.sqrt(2) * 11d);
            deadReckoningDrive.setTargetRotation(0);
            deadReckoningDrive.moveForwardDistance(40d);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(100);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-16);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            double extensionStartingTime = timer.getTime();
//            while(timer.getTime() < extensionStartingTime + 2){
//                motorLeft.setPower(0.5);
//                motorRight.setPower(0.5);
//            }
//            sleep(500);
//            rightWristServo.setPosition(wristVertical);
//            sleep(250);
//            boxServo.setPosition(0.45); // right

        } else {
            deadReckoningDrive.moveForwardDistance(27);
            deadReckoningDrive.moveForwardDistance(-9);
            deadReckoningDrive.moveRightDistance(25);
            deadReckoningDrive.moveForwardDistance(25);

            // Go through stage door
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveForwardDistance(100);

            // Align to backdrop
            deadReckoningDrive.setTargetRotation(90);
            deadReckoningDrive.moveRightDistance(-21);
            deadReckoningDrive.setMotorPowersForTime(2d, 0.2, 0.2, 0.2, 0.2);
//            double extensionStartingTime = timer.getTime();
//            while(timer.getTime() < extensionStartingTime + 2){
//                motorLeft.setPower(0.5);
//                motorRight.setPower(0.5);
//            }
//            sleep(500);
//            rightWristServo.setPosition(wristVertical);
//            sleep(250);
//            boxServo.setPosition(0.45); // right
        }
    }

    @Override
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.BLUE;
    }
}

