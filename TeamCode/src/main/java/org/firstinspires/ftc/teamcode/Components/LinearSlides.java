package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlides {
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;

    int [] leftBounds = {0, 0};
    int [] rightBounds = {0, 0};
    double inPerTickLeft = 0.0;
    double inPerTickRight = 0.0;
    double slidesDistanceDifferenceLimit = 0.0;

    public LinearSlides(HardwareMap hardwareMap) {
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void extendWithPower(double powerLevel){
        if(motorLeft.getCurrentPosition() < leftBounds [0] && powerLevel > 0.0 && // If out of bounds and trying to get back into bounds
                motorRight.getCurrentPosition() < rightBounds [0]){
//                Math.abs(motorLeft.getCurrentPosition() * inPerTickLeft - motorRight.getCurrentPosition() * inPerTickRight) < slidesDistanceDifferenceLimit){

            motorLeft.setPower(powerLevel);
            motorRight.setPower(powerLevel);

        }else if(motorLeft.getCurrentPosition() > leftBounds [1] && powerLevel < 0.0 && // If out of bounds and trying to get back into bounds
                motorRight.getCurrentPosition() > rightBounds [1]){
//                Math.abs(motorLeft.getCurrentPosition() * inPerTickLeft - motorRight.getCurrentPosition() * inPerTickRight) < slidesDistanceDifferenceLimit){

            motorLeft.setPower(powerLevel);
            motorRight.setPower(powerLevel);

        }else if(motorLeft.getCurrentPosition() < leftBounds [0] && motorLeft.getCurrentPosition() < leftBounds [1] && // If out of bounds and trying to get back into bounds
                motorRight.getCurrentPosition() < rightBounds [0] && motorRight.getCurrentPosition() < rightBounds [1]){

            motorLeft.setPower(powerLevel);
            motorRight.setPower(powerLevel);
        }else{ // Last out of bounds case
            motorRight.setPower(0);
            motorLeft.setPower(0);
        }
    }
}
