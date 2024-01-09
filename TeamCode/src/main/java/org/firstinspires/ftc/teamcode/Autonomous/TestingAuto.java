package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

@Autonomous(name = "testing auto")
public class TestingAuto extends LinearOpMode {
    // General
    MecanumDrive drive;
    Odometry odometry;
    PIDDrive PIDDrive;
    ElapsedTime timer;
    double [] driveInputs = {0.0, 0.0, 0.0};
    double [][] targetStates = {
            {0.0, 20.0, Math.PI / 2.0d},
    };
    int numberOfPointsReached = 0;

    // Aligning with tags
    AprilTagLocalizer localizer;
    IMU imu;
    double [] cameraCoordinates = {0.0, 0.0};
    double [] targetCoordinates = {118.5, 35d};
    double aligningP = 0.0;
    double aligningD = 0.0;
    double xError = 0.0;
    double yError = 0.0;
    double lastXError = 0.0;
    double lastYError = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        localizer = new AprilTagLocalizer(hardwareMap, telemetry, 0.0, 0.0);
        timer = new ElapsedTime();
        init_IMU();

        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0d, new Vector2(0.0, 0.0));
        odometry.resetEncoders();
        odometry.setRotation(Math.PI / 2.0d);

        PIDDrive = new PIDDrive(odometry, targetStates [0][0], targetStates [0][1], targetStates [0][2], telemetry);

        waitForStart();

        timer.reset();
        while(timer.milliseconds() < 4000){ // Going forward
            odometry.updatePosition();
            driveInputs = PIDDrive.updatePID();

            drive.FieldOrientedDrive(-driveInputs [0], -driveInputs [1],
                    (odometry.getRotationRadians() - targetStates [numberOfPointsReached][2]) * PIDDrive.P[2], odometry.getRotationRadians(), telemetry);

            telemetry.addData("\nDrive Input 0", -driveInputs [0]);
            telemetry.addData("Drive Input 1", -driveInputs [1]);
            telemetry.addData("Drive Input 2", -driveInputs [2]);

            telemetry.update();
        }

        timer.reset();
        double angularVelocity = 0.0;
        double lastAngle = 0.0;
        while(Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d - 0d) > 5d * Math.PI / 180d){ // Turning right 90˚
            odometry.updatePosition();
            driveInputs = PIDDrive.updatePID();
            telemetry.addData("Angle", (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d) * 180d / Math.PI);
            telemetry.addLine("updated1");
            telemetry.update();

            drive.normalDrive(1d, 0d, 0d, -1.5 * (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d - 0d) +
                    0.3 * ((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d - 0d) - lastAngle));
            lastAngle = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d - 0d);
        }
        Thread.sleep(1000);

        while(Math.sqrt(xError*xError + yError * yError) < 0.25){ // Align to tag 2
            cameraCoordinates = localizer.getRelCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d, 0.0, 0.0);

            xError = targetCoordinates [0] - cameraCoordinates [0];
            yError = targetCoordinates [1] - cameraCoordinates [1];

            double dX = xError - lastXError;
            double dY = yError - lastYError;

            lastXError = xError;
            lastYError = yError;
            drive.FieldOrientedDrive(-0.1 * (xError) + 0.1 * dX,
                    -0.1 * (yError) + 0.1 * dY,
                    0.0,
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d, telemetry);
        }
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
