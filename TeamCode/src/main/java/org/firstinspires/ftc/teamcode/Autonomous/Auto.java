package org.firstinspires.ftc.teamcode.Autonomous;

import android.graphics.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.DefaultMask;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.OuttakeBox;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Coordinates;
import org.firstinspires.ftc.teamcode.Utility.Pose;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public abstract class Auto extends OpMode {
    Odometry odometry;
    Timer timer;
    MecanumDrive mecanumDrive;

    PIDDrive PIDDrive;

    OuttakeBox servoBox;
    Camera camera;
    OpenCvCamera cam;

    Intake intake;


    //camera modes and spike mark
    public CameraEnums.CameraModes BLUE;
    public CameraEnums.SpikeMarkPositions CENTER;

    public int robotPositionNumber = 0;


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
    double [][] targetStates = {{0, 0, 0}};

    double lateralDistance = getDirection()* 24 * getNumTilesToPark();

    OpenCvWebcam webcam;

    int[] dimensions = new int[] { 640, 360 };

    TeamPropMask teamPropMask;

    DefaultMask defaultMask = new DefaultMask();
    boolean isUsingDefault;

    ButtonToggle xToggle;
    ButtonToggle yToggle;

    CameraEnums.SpikeMarkPositions spikeMarkPositions;


    public CameraEnums.CameraModes getColor(){
        return CameraEnums.CameraModes.RED;//by automatic
    }

    @Override
    public void init()
    {
        teamPropMask = new TeamPropMask(dimensions[0], dimensions[1], telemetry);
        telemetry.setAutoClear(false);
        xToggle = new ButtonToggle();
        yToggle = new ButtonToggle();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(teamPropMask);

//
//        spikeMarkPositions = teamPropMask.getPosition();


        robotPositionNumber = getRobotPositionNumber();

        //get the pose that the team prop is on
        Pose poseLine = Coordinates.getSpikeMark(getColor(),spikeMarkPositions);

        telemetry.addLine("Pose" + poseLine);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(dimensions[0], dimensions[1], OpenCvCameraRotation.UPRIGHT);
                // telemetry.addLine("Vector" + teamPropMask.getCoordinates(teamPropPosition,robotPosition));
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error " + errorCode, "error accessing camera stream");
                //   telemetry.addLine("Vector" + teamPropMask.getCoordinates(teamPropPosition,robotPosition));
            }
        });



        timer = new Timer();
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));


        servoBox = new OuttakeBox(hardwareMap, "servoBox");
        intake = new Intake(hardwareMap);
        PIDDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);

        //get the team prop and robot position
        // teamPropPosition  = simpleProcessor.processFrame(frame, 0); //implement a frame: we need to use the camera

        targetStates = setTargetStates();
    }
    public double[][] setTargetStates(){
        return targetStates;
    }



    @Override
    public void loop()
    {
        teamProp();
        pixelCycle();
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

        PIDDrive.setTargetState(teamPropLocation.x, teamPropLocation.y, 3*Math.PI/2); //fix to wanted orient
        while (PIDDrive.distanceToTarget < 0.1){
            //wait
        }
        intake.setMotorPower(-1);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        intake.setMotorPower(0);

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


    public int getPositionNumber(){
        return 0;
    };
}
