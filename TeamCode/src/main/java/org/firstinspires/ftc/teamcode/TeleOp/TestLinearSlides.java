package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.LinearSlides;

@TeleOp(name = "test linear slides")
public class TestLinearSlides extends OpMode {
    LinearSlides slides;

    @Override
    public void init() {
        slides = new LinearSlides(hardwareMap);
    }

    @Override
    public void loop() {
        slides.extendWithPower(-gamepad2.right_stick_y);
    }
}
