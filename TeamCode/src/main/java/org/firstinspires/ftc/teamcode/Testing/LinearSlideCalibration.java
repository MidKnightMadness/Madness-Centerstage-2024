package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class LinearSlideCalibration extends OpMode {
    public DcMotorEx motorRight;
    public DcMotorEx motorLeft;

    int extendedPosition;
    int retractPosition;

    public void init() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetEncoders();
    }

    public void loop() {
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
        int tickPosition = retractPosition + (int) ((extendedPosition - retractPosition) * percent);
        motorRight.setTargetPosition(tickPosition);
        motorLeft.setTargetPosition(tickPosition);
    }
}
