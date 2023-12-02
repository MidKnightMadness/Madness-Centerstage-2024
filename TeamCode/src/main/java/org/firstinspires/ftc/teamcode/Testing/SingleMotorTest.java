package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Single Motor Test")
public class SingleMotorTest extends OpMode {
    public DcMotorEx testMotor;
    public double setRPM = 0.0;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "Test Motor");
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        testMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        if (this.gamepad1.x) {
            testMotor.setPower(1);
        }
        if (gamepad1.y) {
            testMotor.setPower(-1);
        }

        if (gamepad1.a) {
            testMotor.setPower(0);
        }

        telemetry.addData("Power", testMotor.getPower());
    }
}