package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoBack extends Auto {

    @Override
    public int getDirection() {
        return 1;
    }

    @Override
    public int getNumTilesToPark() {
        return 4;
    }
}
