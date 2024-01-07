package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;

;



@TeleOp(name = "New Localization Testing", group = "testing")
public class LocalizationTester extends OpMode {
    // AprilTagLocalizerTwo object and hardware
    AprilTagLocalizer localizer;
    IMU imu;
    MecanumDrive drive;


    double [] cameraCoordinates = {0.0, 0.0};
    double [] targetCoordinates = {118.5, 35d};
    double P = 0.0;
    double D = 0.0;


    @Override
    public void init() {
        localizer = new AprilTagLocalizer(hardwareMap, telemetry, 0.0, 0.0);
        drive = new MecanumDrive(hardwareMap, telemetry);

        init_IMU();
    }

    void init_IMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    double lastXError = 0.0;
    double lastYError = 0.0;
    @Override
    public void loop() {
        if(gamepad1.dpad_up && !gamepad1.dpad_down){
            P += 0.01;
        }else if(!gamepad1.dpad_up && gamepad1.dpad_down) {
            P -= 0.01;
        }
        D += 0.005 * gamepad1.right_stick_y;

        cameraCoordinates = localizer.getRelCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), 0.0, 0.0);
        double xError = targetCoordinates [0] - cameraCoordinates [0];
        double yError = targetCoordinates [1] - cameraCoordinates [1];

        double dX = xError - lastXError;
        double dY = yError - lastYError;

        lastXError = xError;
        lastYError = yError;
        drive.FieldOrientedDrive(P * (xError) - D * dX,
                P * (yError) - D * dY,
                0.0,
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), telemetry);

        telemetry();
    }

    public void telemetry(){
        telemetry.addData("imu yaw", getRobotDegrees());
        if(cameraCoordinates != null){
            telemetry.addData("Percieved x", cameraCoordinates [0]);
            telemetry.addData("Percieved y", cameraCoordinates [1]);
        }else{
            telemetry.addLine("No detections");
        }

        telemetry.addData("P", P);
        telemetry.addData("D", D);
        telemetry.addLine("\n\n\n========================");

        localizer.telemetryAprilTag();

        telemetry.update();
    }

    double getRobotDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
