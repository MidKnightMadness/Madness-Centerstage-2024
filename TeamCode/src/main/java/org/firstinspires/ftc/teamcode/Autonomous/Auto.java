package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

@TeleOp
public class Auto extends OpMode {
    Odometry odometry;

    public int getDirection() {
        return -1;
    }

    public int getNumTilesToPark() {
        return 4;
    }

    double lateralDistance = getDirection()* 24 * getNumTilesToPark();



    @Override
    public void init()
    {
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));
    }

    @Override
    public void loop()
    {
        park();
    }

    void park() {

    }

}
