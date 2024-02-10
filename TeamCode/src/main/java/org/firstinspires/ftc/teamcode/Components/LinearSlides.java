package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Timer;

public class LinearSlides{
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;
    public double [] slidesMotorMultipliers = {0.75, 0.75}; // Left, right

    ButtonToggle dPadUp;
    ButtonToggle dPadDown;

    ButtonToggle y;
    ButtonToggle a;

    double [] mainExtensionConstants = {0.1, 0.1}; // For both sides to follow based on distance to target; left, right
    double correctionConstant = 0.0; // Left slide follows right side, to help witn synchronization
    int [] leftBounds = {0, -2629}; // Bottom, top
    int [] rightBounds = {0, 2974}; // Bottom, top
    int [] startingPositions = {0, 0}; // Left, right
    double inPerTickLeftSlide = -21.5 / 2629d;
    double inPerTickRightSlide = 21.5 / 2974d;
    double slidesDifferenceTolerance = 0.0; // Length difference between two slides tolerated
    double slidesExtensionTolerance = 0.25;

    double targetPos; // Inches
    double currentPos; // Inches from starting length
    boolean movementToggled = false;

    Timer timer;

    public LinearSlides(HardwareMap hardwareMap) {
        timer = new Timer();
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");


        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dPadUp = new ButtonToggle();
        dPadDown = new ButtonToggle();

//        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        toggleTargetExtension();
    }

    public LinearSlides(HardwareMap hardwareMap, int leftPos, int rightPose) {
        timer = new Timer();
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dPadUp = new ButtonToggle();
        dPadDown = new ButtonToggle();

//        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorRight.setMode(DcMotor.RunMhode.RUN_WITHOUT_ENCODER);

        setStartingPositions(motorLeft.getCurrentPosition(), motorRight.getCurrentPosition());

        toggleTargetExtension();
    }

    public void setStartingPositions(int leftPos, int rightPos){
        startingPositions [0] = leftPos;
        startingPositions [1] = rightPos;
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
//        if(motorLeft.getCurrentPosition() < startingPositions [0] && motorLeft.getCurrentPosition() > startingPositions [0] + leftBounds [1] &&
//        motorRight.getCurrentPosition() > startingPositions [1] && motorRight.getCurrentPosition() < startingPositions [1] + rightBounds [1] &&
//                (leftPowerInput + rightPowerInput) / 2d > 0) {
        if(true){
            motorLeft.setPower(leftPowerInput * slidesMotorMultipliers[0]);
            motorRight.setPower(rightPowerInput * slidesMotorMultipliers[1]);
        }
    }

    public void extendForTime(double left, double right, double seconds) {
        double startTime = timer.updateTime();

        // run for time
        while (timer.getTime() - startTime < seconds) {
            motorLeft.setPower(left);
            motorRight.setPower(right);

            timer.updateTime();
        }

        motorRight.setPower(0);
        motorLeft.setPower(0);
    }


    public void update(double targetPosition){ // Run this each tick to set power based on position differences
        targetPos = targetPosition;
        currentPos = (inPerTickRightSlide * (motorRight.getCurrentPosition() - startingPositions [1]) - inPerTickLeftSlide * (motorLeft.getCurrentPosition() * startingPositions [0])) / 2.0d;
    }

    public void extendToDistance(double targetDistance){
        this.update(targetDistance);
        while(this.targetPos - this.currentPos > slidesExtensionTolerance){
            this.update(targetDistance);
            if(movementToggled){
                motorRight.setPower((targetDistance - currentPos) * mainExtensionConstants [0]);
                motorLeft.setPower((targetDistance - currentPos) * mainExtensionConstants [1] +
                        correctionConstant * (inPerTickLeftSlide * motorRight.getCurrentPosition() - inPerTickRightSlide * motorLeft.getCurrentPosition())); // For following
            }else{
                motorRight.setPower(0.0);
                motorLeft.setPower(0.0);
            }
        }
    }

    public void extendToDistanceAsync(double targetDistance){ // "Async", only allows extension while driving; run every frame
        this.update(targetDistance);
        if(this.targetPos - this.currentPos > slidesExtensionTolerance){
            this.update(targetDistance);
            if(movementToggled){
                motorRight.setPower((targetDistance - currentPos) * mainExtensionConstants [0]);
                motorLeft.setPower((targetDistance - currentPos) * mainExtensionConstants [1] +
                        correctionConstant * (inPerTickLeftSlide * motorRight.getCurrentPosition() - inPerTickRightSlide * motorLeft.getCurrentPosition())); // For following
            }else{
                motorRight.setPower(0.0);
                motorLeft.setPower(0.0);
            }
        }
    }
}
