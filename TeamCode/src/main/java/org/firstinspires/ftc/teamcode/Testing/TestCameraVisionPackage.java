package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.CameraLocalizationPackage;

@TeleOp(name = "Test Camera Vision Package Class", group = "concept")
public class TestCameraVisionPackage extends OpMode {
    CameraLocalizationPackage localizationPackage;
    MecanumDrive mecanumDrive;
    IMU imu;

    // For alignment with camera
    double P = 0.1;
    double D = 0.075;
    double [] perceivedPosition = {0.0, 0.0};
    double [] targetCoordinates = {118.5, 109d};
    double [] deltaPosition = {0.0, 0.0};
    double [] lastPosition = {0.0, 0.0};
    double [] velocity = {0.0, 0.0};
    final double targetAngle = 0;

    @Override
    public void init() {
        init_IMU();
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        imu.resetYaw();

        localizationPackage = new CameraLocalizationPackage(hardwareMap, telemetry, "Webcam 2");
        localizationPackage.toggleTeamPropActivation();
        localizationPackage.toggleAprilTagActivation();

        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        perceivedPosition = localizationPackage.getCameraCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), perceivedPosition [0], perceivedPosition [1]);
        deltaPosition [0] = targetCoordinates [0] - perceivedPosition [0];
        deltaPosition [1] = targetCoordinates [1] - perceivedPosition [1];
        velocity [0] = perceivedPosition [0] - lastPosition [0];
        velocity [1] = perceivedPosition [1] - lastPosition [1];

        telemetry.addData("Using April Tag Detector", localizationPackage.getAprilTagActivation());
        telemetry.update();
    }

    @Override
    public void loop() {
        perceivedPosition = localizationPackage.getCameraCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), perceivedPosition [0], perceivedPosition [1]);
        deltaPosition [0] = targetCoordinates [0] - perceivedPosition [0];
        deltaPosition [1] = targetCoordinates [1] - perceivedPosition [1];
        velocity [0] = perceivedPosition [0] - lastPosition [0];
        velocity [1] = perceivedPosition [1] - lastPosition [1];

        mecanumDrive.normalDrive(1.0, -P * deltaPosition [1], 0/*-P * deltaPosition[0]*/, 0.2 * (targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));

        lastPosition [0] = perceivedPosition [0];
        lastPosition [1] = perceivedPosition [1];
    }

    public void telemetry(){
        telemetry.addLine(String.format("Delta: [%5.2f, %5.2f]", deltaPosition [1], deltaPosition [1]));
        telemetry.addData("Using April Tag Detector", localizationPackage.getAprilTagActivation());
        localizationPackage.telemetryAprilTag();
        telemetry.update();
    }

    void init_IMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }
}
