package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;

@Autonomous
public class TestIntakeFromStack extends AutoDeadReckoning{
    @Override
    public void drive(){
        intakeRightServo.setPosition(ServoPositions.intakeStackOfFive);
        sleep(250);
        deadReckoningDrive.moveForwardDistance(-2, 0.5);
        sleep(250);
        IntakeMotor.setPower(0.5);
        sleep(250);
        IntakeMotor.setPower(1.0);
        sleep(1000);
        deadReckoningDrive.moveForwardDistance(2, 0.5);
        IntakeMotor.setPower(0);
    }
}
