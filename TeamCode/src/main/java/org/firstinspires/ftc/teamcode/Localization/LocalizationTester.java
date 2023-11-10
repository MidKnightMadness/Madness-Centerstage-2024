package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "New Localization Testing")
public class LocalizationTester extends OpMode {
    // AprilTagLocalizerTwo object and hardware
    AprilTagLocalizerTwo localizer;
    BNO055IMU imu;

    // Auxillary Variables
    BNO055IMU.Parameters parameters;
    Orientation angles;


    @Override
    public void init() {
        localizer = new AprilTagLocalizerTwo(hardwareMap, telemetry, 0.0, 0.0);

        parameters = new BNO055IMU.Parameters();

    }

    @Override
    public void loop() {

    }
}
