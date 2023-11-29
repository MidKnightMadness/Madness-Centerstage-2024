package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;

import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;
import org.firstinspires.ftc.teamcode.Utility.Vector2;;



@TeleOp(name = "New Localization Testing")
public class LocalizationTester extends OpMode {
    // AprilTagLocalizerTwo object and hardware
    AprilTagLocalizerTwo localizer;
//    BNO055IMU imu;
    Odometry odometry;

    // Auxillary Variables
//    BNO055IMU.Parameters parameters;
//    Orientation angles;
    double [] cameraCoordinates = {0.0, 0.0};


    @Override
    public void init() {
        localizer = new AprilTagLocalizerTwo(hardwareMap, telemetry, 0.0, 0.0);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "imu";
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        // 4 in. from either side of F4 tile
        odometry = new Odometry(hardwareMap, Math.PI / 2.0d, new Vector2(83.5, 6.75));
        odometry.resetEncoders();
        odometry.position = new Vector2(83.5, 6.75);
        odometry.rotationRadians = Math.PI / 2.0d;
    }

    @Override
    public void loop() {
        odometry.updatePosition();

        cameraCoordinates = localizer.getRelCoords(odometry.getRotationRadians(), 0.0, 0.0);

        telemetry();
    }

    public void telemetry(){
        telemetry.addData("\nRobot heading", odometry.getRotationDegrees());
        telemetry.addData("x", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());

        telemetry.addData("\nleft ticks", odometry.leftEncoder.getCurrentPosition());
        telemetry.addData("right ticks", odometry.rightEncoder.getCurrentPosition());
        telemetry.addData("center ticks", odometry.horizontalEncoder.getCurrentPosition());

        if(cameraCoordinates != null){
            telemetry.addData("Percieved x", cameraCoordinates [0]);
            telemetry.addData("Percieved y", cameraCoordinates [1]);
        }else{
            telemetry.addLine("No detections");
        }
        telemetry.addLine("\n\n\n========================");

        localizer.telemetryAprilTag();

        telemetry.update();
    }
}
