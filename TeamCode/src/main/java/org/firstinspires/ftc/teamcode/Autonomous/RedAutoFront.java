package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedAutoFront extends Auto {

    final int positionNumber = 1;
    @Override
    public int getDirection() {
        return 1;
    }

    @Override
    public int getNumTilesToPark() {
        return 2;
    }

    @Override
    public int getPositionNumber(){
        return positionNumber;//Position Number is 1
    }
}
