package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizer;


@TeleOp
public class TestAprilTag extends OpMode {
    AprilTagLocalizer aprilTagLocalizer;

    @Override
    public void init() {
        aprilTagLocalizer = new AprilTagLocalizer(hardwareMap, telemetry, 0,0,0);

    }

    @Override
    public void loop() {
        aprilTagLocalizer.telemetryAprilTag();
        telemetry.update();
    }
}




