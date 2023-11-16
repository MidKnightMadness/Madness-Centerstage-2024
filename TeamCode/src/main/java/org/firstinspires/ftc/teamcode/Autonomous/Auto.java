package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Localization.SimpleProcessor;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
public class Auto extends OpMode {
    Odometry odometry;
    Timer timer;
    MecanumDrive mecanumDrive;

    SimpleProcessor simpleProcessor;
    DcMotor intakeMotor;

    Camera camera;
    OpenCvCamera cam;


    Vector2 teamPropLocation = new Vector2(0,0);
    public int getDirection() {
        return -1;
    }

    public int getNumTilesToPark() {
        return 4;
    }

    public int getRobotPositionNumber(){
        return 1;
    }
    public int teamPropPosition = 3;
    public int robotPositionNumber = 1;

    double lateralDistance = getDirection()* 24 * getNumTilesToPark();

    @Override
    public void init()
    {

        cam.setPipeline(simpleProcessor.processFrame());
        timer = new Timer();
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));
        simpleProcessor = new SimpleProcessor();

        //get the team prop and robot postion
        teamPropPosition  = simpleProcessor.processFrame(frame, 0);
        robotPositionNumber = getRobotPositionNumber();

        //get the vector that the team prop is on
        teamPropLocation = simpleProcessor.getVector(teamPropPosition,robotPositionNumber);

    }

    @Override
    public void loop()
    {
        park();
    }

    void park() {
        // drive forward for one second
        drive(0.25, new Vector2(0, 1), 0.5);

        // drive left/right for (getNumTiles * 2) seconds
        drive(getNumTilesToPark() * 2, new Vector2(getDirection(), 0), 0.5);

        // reverse intake preloaded pixels?
    }

    void drive(double seconds, Vector2 direction, double power) {
        timer.updateTime();
        double startTime = timer.getTime();
        Vector2 normalizedDirection = direction.getNormalized();
        while (timer.getTime() - startTime < seconds) {
            mecanumDrive.normalDrive(power, normalizedDirection.x, normalizedDirection.y, 0);
            timer.updateTime();
        }
    }



}
