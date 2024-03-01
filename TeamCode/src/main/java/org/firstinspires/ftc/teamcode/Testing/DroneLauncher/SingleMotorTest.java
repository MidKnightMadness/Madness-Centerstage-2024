package org.firstinspires.ftc.teamcode.Testing.DroneLauncher;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Single Motor Test", group = "testing")
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

        if (gamepad1.x) {
            testMotor.setPower(1);
        }
        if (gamepad1.y) {
            testMotor.setPower(-1);
        }

        if (gamepad1.a) {
            testMotor.setPower(0);
        }

        if(! gamepad1.x && ! gamepad1.y && ! gamepad1.a){
            testMotor.setPower(gamepad1.right_stick_y);
        }

        telemetry.addData("Power", testMotor.getPower());
    }
}