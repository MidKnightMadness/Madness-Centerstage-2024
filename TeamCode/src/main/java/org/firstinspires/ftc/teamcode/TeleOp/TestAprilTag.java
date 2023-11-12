package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;


@TeleOp(name = "TestAprilTag")
@Disabled
public class TestAprilTag extends OpMode {
    AprilTagLocalizer aprilTagLocalizer;

    BNO055IMU imu;

    Orientation angles;

    Float robotHeading;

    Float currentX;
    Float currentY;

    public double [][] coordinates =
            { //coordinates is 10 id arrays each with an array of 2(x and y)
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0},
                    {0.0, 0.0}
            };

    @Override
    public void init() {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, telemetry, 0, 0);


        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();//set imu parameters
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;//set imu angle unit to degrees
        imuParameters.loggingEnabled = true;
        imuParameters.loggingTag = "imu";
        imuParameters.calibrationDataFile = "BN0055IMUCalibration.json";//calibrate imu

        imu = hardwareMap.get(BNO055IMU.class,"imu");//set imu to hardware map
        imu.initialize(imuParameters);//initialize imu to imu parameters

       /* coordinates = new double [10][];
        for(int i = 0; i < coordinates.length; i++){
            coordinates [i] = new double [2];
        }
        */

    }

    @Override
    public void loop() {
        double averageX = 0;
        double averageY = 0;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //set imu values to the values received

        //pass on the variables with the imu values
        robotHeading = angles.firstAngle;
        currentX = angles.thirdAngle;
        currentY = angles.secondAngle;

        //pass on values into getRelativeCoordinates function
//        coordinates = aprilTagLocalizer.getCoordsSet(robotHeading, currentX, currentY);
        int numberOfAprilTags = 0;
        if (coordinates != null){
            for(int i = 0;i<10;i++){
                if(coordinates[i][0] != 0.0) {
                    telemetry.addLine(String.format("Coordinates of April Tag(ID # %d [%3.2f, %3.2f] ", i + 1, coordinates[i][0], coordinates[i][1]));
                    numberOfAprilTags++;
                }
            }
        }


        if(coordinates[3][0]!=0&&coordinates[4][0]!=0&&coordinates[5][0]!=0){
            averageX = (coordinates[3][0]+coordinates[4][0] + coordinates[5][0])/3;
            averageY = (coordinates[3][1] + coordinates[4][1] + coordinates[5][1])/3;
        }
        else if(coordinates[0][0]!=0&&coordinates[1][0]!=0&&coordinates[2][0]!=0){
            averageX = (coordinates[0][0]+coordinates[1][0] + coordinates[2][0])/3;
            averageY = (coordinates[0][1] + coordinates[1][1] + coordinates[2][1])/3;
        }

        telemetry.addLine(String.format("Average of 3 april tags: [%3.2f, %3.2f]", averageX, averageY));


        telemetry.addData("Robot heading", angles.firstAngle);//uses imu to get the robot heading
        aprilTagLocalizer.telemetryAprilTag();
        telemetry.update();

        for(int i = 0;i<10;i++){
            coordinates[i][0] = 0;
            coordinates[i][1] = 0;
        }


    }
}


