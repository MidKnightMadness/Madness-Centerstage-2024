package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

@TeleOp
public class Auto extends OpMode {
    Odometry odometry;
    Timer timer;
    MecanumDrive mecanumDrive;

    DcMotor intakeMotor;

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
        timer = new Timer();
        odometry = new Odometry(hardwareMap, 0, new Vector2<Double>(0, 0));
    }

    @Override
    public void loop()
    {
        park();
    }

    void park() {
        // drive forward for one second
        drive(0.25, new Vector2<Double>(0, 1), 0.5);

        // drive left/right for (getNumTiles * 2) seconds
        drive(getNumTilesToPark() * 2, new Vector2<Double>(getDirection(), 0), 0.5);

        // reverse intake preloaded pixels?
    }

    void drive(double seconds, Vector2<Double> direction, double power) {
        timer.updateTime();
        double startTime = timer.getTime();
        Vector2<Double> normalizedDirection = direction.getNormalized();
        while (timer.getTime() - startTime < seconds) {
            mecanumDrive.normalDrive(power, normalizedDirection.x, normalizedDirection.y, 0);
            timer.updateTime();
        }
    }

}
