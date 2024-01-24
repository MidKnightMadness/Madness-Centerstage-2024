package org.firstinspires.ftc.teamcode.Testing;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.DeadReckoningDrive;
import org.firstinspires.ftc.teamcode.Autonomous.StartingPosition;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.CameraModes;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.SpikeMarkPositions;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Components.LinearSlides;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/* For testing opmodes without odometry
Objectives:
1. Driving forward by distance, whether by wheel radii calculations or by calibrated time and power calculations
2. Smooithly ramp up and ramp down power
Contains:
1. Drive using power and time (primarily used)
2. Drive using distance
3. Conversion functions and compensation for RPM-torque differences between motors
 */
@TeleOp(name = "camera full procedure test", group = "testing")
@SuppressLint("DefaultLocale")
public class CameraFullProcedureTest extends OpMode {
    public CameraModes getAllianceColor(){
        return CameraModes.RED;
    }

    public StartingPosition getStartingPosition() {
        return StartingPosition.NEAR;
    }

    CameraModes cameraMode = getAllianceColor();
    IMU imu;
    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;
    Timer timer;
    OpenCvWebcam webcam;
    public WebcamName webcamName;
    TeamPropMask teamPropMask;
    AprilTagLocalizerTwo localizer;

    @Override
    public void init() {
        timer = new Timer();
        telemetry.setAutoClear(false);
        init_IMU();

        teamPropMask = new TeamPropMask(640, 360, telemetry);
        teamPropMask.setMode(getAllianceColor());

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(teamPropMask);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error " + errorCode, "error accessing camera stream");
            }
        });

        // Init for april tag localizer
        localizer = new AprilTagLocalizerTwo(webcamName, hardwareMap, telemetry, 0, 0);
    }

    @Override
    public void init_loop() {
        telemetry.clear();
        teamPropPosition = teamPropMask.getSpikeMarkPosition();
        telemetry.addData("Detected spike mark position", teamPropPosition);
        if(cameraMode == CameraModes.RED){
            telemetry.addLine("Camera Mode: RED");
        }else{
            telemetry.addLine("Camera Mode: BLUE");
        }
    }

    @Override
    public void start() {
        webcam.stopStreaming();

        // Init
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while(true){
            telemetry.clear();
            telemetry.addData("Time", timer.seconds());

            sleep(500);
//            localizer.telemetryAprilTag();
            telemetry.update();
        }
    }



    void sleep(long milis) {
        try {
            Thread.sleep(milis);
        }
        catch (InterruptedException e) {
            telemetry.addData("Error", e.getMessage());
        }

    }
    double increment = 0.0005;
    double kp = 0.2;

    @Override
    public void loop() {
    }

    void init_IMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    double getRobotDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

}
