package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
/* Full outtake class for easier usage
Objectives:
1. Combine functionality to ensure box doesn't collide
2. Add presets
3. Position robot to properly align box with backdrop
4.
Contains:
1.
 */
@TeleOp(name = "outtake testing", group = "testing")
public class Outtake extends OpMode {
    // Parts
    OuttakeBox endEffector;
    LinearSlides slides;
    int stage = 0;


    public Outtake(HardwareMap hardwareMap){
        endEffector = new OuttakeBox(hardwareMap, "Center box servo");
        slides = new LinearSlides(hardwareMap);
    }

    @Override
    public void init() {
        endEffector = new OuttakeBox(hardwareMap, "Center box servo");
        slides = new LinearSlides(hardwareMap);
    }

    @Override
    public void loop() {
        slides.extendWithPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        telemetry.addData("Left side power", slides.motorLeft.getPower());

    }
}
