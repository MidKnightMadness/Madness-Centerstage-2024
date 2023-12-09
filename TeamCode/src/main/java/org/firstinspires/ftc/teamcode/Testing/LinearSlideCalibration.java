package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp
public class LinearSlideCalibration extends OpMode {
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;
//    public Servo ElbowJoint;
//    public Servo WristJoint;

    ButtonToggle dPadUp;
    ButtonToggle dPadDown;

    ButtonToggle y;
    ButtonToggle a;

    int[] leftBounds = {};
    int[] rightBounds = {};

    int targetPos;
    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        ElbowJoint = hardwareMap.get(Servo.class, "");

        dPadUp = new ButtonToggle();
        dPadDown = new ButtonToggle();

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
//        if (dPadUp.update(this.gamepad1.y)) {
//            targetPos += 500;
//        }
//        if (dPadDown.update(this.gamepad1.a)) {
//            targetPos -= 500;
//        }
//
        motorLeft.setPower(this.gamepad1.left_stick_y * 0.3);
        motorRight.setPower(this.gamepad1.right_stick_y * 0.3);
//
//
//        motorLeft.setTargetPosition(targetPos);
//        motorRight.setTargetPosition(targetPos);


        telemetry.addData("Target pos", targetPos);
        telemetry.addData("Left motor Power", motorLeft.getPower());
        telemetry.addData("Right motor Power", motorRight.getPower());

        telemetry.addData("Left motor", motorLeft.getCurrentPosition());
        telemetry.addData("Right motor", motorRight.getCurrentPosition());
    }


    public void resetEncoders() {
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void setExtension(double percent) {

    }
}
