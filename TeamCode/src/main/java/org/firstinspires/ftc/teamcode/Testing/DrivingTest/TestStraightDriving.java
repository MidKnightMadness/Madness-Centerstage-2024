package org.firstinspires.ftc.teamcode.Testing.DrivingTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutoDeadReckoning;

@Autonomous(name = "Test straight driving")
public class TestStraightDriving extends AutoDeadReckoning {

    @Override
    public void drive(){
        deadReckoningDrive.moveForwardDistance(50d, 0.5, 0);
        deadReckoningDrive.moveRightDistance(-20d, 0);
    }
}
