package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class OuttakeTesting extends OpMode {
    public DcMotor LeftLinearSlides;
    public DcMotor RightLinearSlides;

    public Servo box = null;



    public final static double BOX_HOME = 0.25; // Starting position for box


    @Override
    public void init() {
        box.setPosition(BOX_HOME);
        box = hardwareMap.get(Servo.class,"Box");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            box.setPosition(0.5);
        } else if (gamepad1.left_bumper) {
            box.setPosition(0);
        }
        else {
            box.setPosition(BOX_HOME);
        }
    }
}
