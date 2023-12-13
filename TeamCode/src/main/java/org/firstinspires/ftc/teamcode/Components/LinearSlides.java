package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

public class LinearSlides{
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;
    public double [] slidesMotorMultipliers = {0.25, 0.3}; // Left, right

    ButtonToggle dPadUp;
    ButtonToggle dPadDown;

    ButtonToggle y;
    ButtonToggle a;

    int [] leftBounds = {0, 0}; // Bottom, top
    int [] rightBounds = {0, 0}; // Bottom, top
    double inPerTickLeftSlide = 0.0;
    double inPerTickRightSlide = 0.0;
    double [] mainExtensionConstants = {0.0, 0.0}; // For both sides to follow based on distance to target; left, right
    double correctionConstant = 0.0; // Left slide follows right side, to help witn synchronization
    double targetPos; // Inches from starting length
    double currentPos; // Inches from starting length
    boolean movementToggled = false;

    public LinearSlides(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dPadUp = new ButtonToggle();
        dPadDown = new ButtonToggle();

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        toggleTargetExtension();
    }

    public void resetEncoders() {
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleTargetExtension(){
        movementToggled = !movementToggled;
    }

    public void engageMotors(){
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void disengageMotors(){
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void extendWithPower(double leftPowerInput, double rightPowerInput){
        motorLeft.setPower(leftPowerInput * slidesMotorMultipliers [0]);
        motorRight.setPower(rightPowerInput * slidesMotorMultipliers [1]);
    }

    public void update(double targetPosition){ // Run this each tick to set power based on position differences
        targetPos = targetPosition;
        currentPos = (inPerTickRightSlide * motorRight.getCurrentPosition() + inPerTickLeftSlide * motorLeft.getCurrentPosition()) / 2.0d;

        if(movementToggled){
            motorRight.setPower((targetPosition - currentPos) * mainExtensionConstants [0]);
            motorLeft.setPower((targetPosition - currentPos) * mainExtensionConstants [1] +
                    correctionConstant * (inPerTickLeftSlide * motorRight.getCurrentPosition() - inPerTickRightSlide * motorLeft.getCurrentPosition())); // For following
        }else{
            motorRight.setPower(0.0);
            motorLeft.setPower(0.0);
        }
    }
}
