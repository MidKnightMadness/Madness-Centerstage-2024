package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Single Motor Test")
@Disabled
public class SingleMotorTest extends OpMode {
    public DcMotorEx testMotor;
    public double setRPM = 0.0;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "Test Motor");
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y != 0){
            setRPM += gamepad1.left_stick_y * 10;
        }

        if (gamepad1.right_trigger != 0) {
            setRPM = 0;
        }

        testMotor.setVelocity(setRPM * 30f / Math.PI, AngleUnit.RADIANS);
        telemetry.addData("Measured RPM", testMotor.getVelocity(AngleUnit.RADIANS) * 30f / Math.PI);
        telemetry.addData("Desired RPM: ", setRPM);
    }
}