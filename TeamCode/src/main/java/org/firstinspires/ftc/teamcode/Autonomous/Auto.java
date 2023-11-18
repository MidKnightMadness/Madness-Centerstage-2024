package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.OuttakeBox;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Localization.SimpleProcessor;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp
public class Auto extends OpMode {
    Odometry odometry;
    Timer timer;
    MecanumDrive mecanumDrive;

    SimpleProcessor simpleProcessor;
    PIDDrive PIDDrive;
    DcMotor intakeMotor;
    OuttakeBox servoBox;
    Camera camera;
    OpenCvCamera cam;

    Intake intake;

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
    double [][] targetStates = {{0, 0, 0}};

    double lateralDistance = getDirection()* 24 * getNumTilesToPark();
    public int getPositionNumber(){
        return robotPositionNumber;
    }
    @Override
    public void init()
    {

        cam.setPipeline(simpleProcessor.processFrame());
        timer = new Timer();
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));
        simpleProcessor = new SimpleProcessor();


        servoBox = new OuttakeBox(hardwareMap, "servoBox");
        intake = new Intake(hardwareMap);
        PIDDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);

        //get the team prop and robot postion
        teamPropPosition  = simpleProcessor.processFrame(frame, 0); //implement a frame: we need to use the camera
        robotPositionNumber = getRobotPositionNumber();

        //get the vector that the team prop is on
        teamPropLocation = simpleProcessor.getVector(teamPropPosition, getPositionNumber());
        targetStates = setTargetStates();
    }
    public double[][] setTargetStates(){
        return targetStates;
    }

    @Override
    public void loop()
    {

        teamProp();
        park();

    }

    void park() {
        // drive forward for one second
        drive(0.25, new Vector2(0, 1), 0.5);

        // drive left/right for (getNumTiles * 2) seconds
        drive(getNumTilesToPark() * 2, new Vector2(getDirection(), 0), 0.5);

        // reverse intake preloaded pixels?
    }

    void teamProp(){

        PIDDrive.setTargetState(teamPropPosition, 3*Math.PI/2); //fix to wanted orient
        while (PIDDrive.distanceToTarget < 0.1){
            //wait
        }
        intake.setMotorPower(-1);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }
    void pixelCycle() {

        //{preset coordinates}
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
