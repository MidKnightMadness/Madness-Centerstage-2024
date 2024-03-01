package org.firstinspires.ftc.teamcode.Testing.Slides;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp(name = "linear slides calibration", group = "testing")
public class LinearSlideCalibration extends OpMode {
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;
    public Servo ElbowJoint;
    public Servo WristJoint;

    ButtonToggle dPadUp;
    ButtonToggle dPadDown;

    ButtonToggle y;
    ButtonToggle a;

    int [] leftBounds = {0, 0}; // Bottom, top
    int [] rightBounds = {0, 0}; // Bottom, top
    int [] startingPositions = {0, 0}; // Left, right
    double inPerTickLeftSlide = 0.0;
    double inPerTickRightSlide = 0.0;
    double slidesDifferenceTolerance = 0.0; // Length difference between two slides tolerated
    double extensionLength = 0.0; // Extended length, use length of right side (lead side)
    double correctionConstant = 0.0; // Left slide follows right side, to help witn synchronization

    int targetPos;

    double rightSideMultiplier = 0.75; // Default
    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        double both = -gamepad1.left_trigger + gamepad1.right_trigger;
        motorLeft.setPower((gamepad1.left_stick_y + both) * 0.5);
        motorRight.setPower((gamepad1.right_stick_y + both)* 0.5 * rightSideMultiplier);

        if (this.gamepad1.right_bumper) {
            resetEncoders();
        }
//
//
//        motorLeft.setTargetPosition(targetPos);
//        motorRight.setTargetPosition(targetPos);

        if(gamepad1.dpad_up && !gamepad1.dpad_down){
            rightSideMultiplier += 0.005;
        }else if(!gamepad1.dpad_up && gamepad1.dpad_down){
            rightSideMultiplier -= 0.005;
        }

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
