package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "testing", name = "Autonomous Slides Testing")
public class SlidesTesting extends LinearOpMode {
    public DcMotorEx motorRight, motorLeft;
    Servo boxServo, rightElbowServo, rightWristServo;
    ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        rightElbowServo = hardwareMap.get(Servo.class, "Right elbow servo");
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");
        boxServo = hardwareMap.get(Servo.class, "Center box servo");
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();
        timer.startTime();

        telemetry.addLine("Initialized");
        telemetry.update();

        // Set hardware in correct positions
        boxServo.setPosition(0.6435); // center
        rightWristServo.setPosition(0.388); // down

        waitForStart();

        timer.reset();
        while(timer.milliseconds() < 5000){
            motorLeft.setPower(-0.25);
            motorRight.setPower(-0.25 * 0.75);

            if(timer.milliseconds() > 1000){
                rightWristServo.setPosition(0.58); // vertical

                if(timer.milliseconds() > 1500) {
                    boxServo.setPosition(0.45); // right
                }else{
                    boxServo.setPosition(0.6435); // center
                }
            }else{
                boxServo.setPosition(0.6435); // center
                rightWristServo.setPosition(0.388); // down
            }
        }

        boxServo.setPosition(0.6435); // center
        rightWristServo.setPosition(0.388); // down
        motorLeft.setPower(-0.25);
        motorRight.setPower(-0.25 * 0.75);

        Thread.sleep(1000);
        telemetry.addLine("Waiting to go down");
        telemetry.update();

        timer.reset();
        telemetry.addLine("Retracting");
        telemetry.update();
        while(timer.milliseconds() < 5000){
            motorLeft.setPower(0.2);
            motorRight.setPower(0.2 * 0.75);

            boxServo.setPosition(0.6435); // center
            rightWristServo.setPosition(0.388); // down
        }
    }
}
