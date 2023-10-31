package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;


@TeleOp(name = "hi")
public class TestAprilTag extends OpMode {
    AprilTagLocalizer aprilTagLocalizer;

    BNO055IMU imu;

    Orientation angles;

    Float robotHeading;

    Float currentX;
    Float currentY;

    double[] coordinates;

    @Override
    public void init() {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, telemetry, 0, 0, 0);

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();//set imu parameters
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//set imu angle unit to degrees
        imu = hardwareMap.get(BNO055IMU.class,"imu");//set imu to hardware map
        imuParameters.calibrationDataFile = "BN0055IMUCalibration.json";//calibrate imu
        imu.initialize(imuParameters);//initialize imu to imu parameters

        coordinates = new double[]{0.0, 0.0};
    }

    @Override
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //set imu values to the values received

        //pass on the variables with the imu values
        robotHeading = angles.firstAngle;
        currentX = angles.thirdAngle;
        currentY = angles.secondAngle;

        //pass on values into getRelativeCoordinates function
        coordinates = aprilTagLocalizer.getRelCoords(robotHeading, currentX, currentY);

        if (coordinates != null){
        telemetry.addLine(String.format("Coordinates of April Tag: [%f, %f] ", coordinates[0], coordinates[1]));
        }
        aprilTagLocalizer.telemetryAprilTag();
        telemetry.update();
    }
}




