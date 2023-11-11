package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;

import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;;



@TeleOp(name = "New Localization Testing")
public class LocalizationTester extends OpMode {
    // AprilTagLocalizerTwo object and hardware
    AprilTagLocalizerTwo localizer;
    BNO055IMU imu;

    // Auxillary Variables
    BNO055IMU.Parameters parameters;
    Orientation angles;
    double [] cameraCoordinates = {0.0, 0.0};


    @Override
    public void init() {
        localizer = new AprilTagLocalizerTwo(hardwareMap, telemetry, 0.0, 0.0);

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "imu";
        parameters.calibrationDataFile = "BN0055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        cameraCoordinates = localizer.getRelCoords(angles.thirdAngle, 0.0, 0.0);

        telemetry();
    }

    public void telemetry(){
        telemetry.addData("Robot heading", angles.thirdAngle * 180 / Math.PI);
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
