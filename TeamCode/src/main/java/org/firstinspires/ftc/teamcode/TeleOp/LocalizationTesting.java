package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;

@TeleOp(name = "localization testing")
public class LocalizationTesting extends OpMode{
    AprilTagLocalizerTwo localizer;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;


    @Override
    public void init() {
        localizer = new AprilTagLocalizerTwo(hardwareMap, telemetry, 0.0, 0.0);

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        localizer.getRelCoords(angles.thirdAngle, 0.0, 0.0);

        telemetry.addData("SensorCoordinates", localizer.sensorCoordinates);
        telemetry.addData("Robot heading", angles.thirdAngle * 180 / Math.PI);
    }
}