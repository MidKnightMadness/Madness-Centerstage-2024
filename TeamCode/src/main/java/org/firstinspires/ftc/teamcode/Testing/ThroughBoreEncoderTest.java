package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utility.AverageBuffer;
import org.firstinspires.ftc.teamcode.Utility.Timer;

@TeleOp
@Disabled
public class ThroughBoreEncoderTest extends OpMode {
    DcMotor encoder1;
    DcMotor encoder2;
    DcMotor encoder3;
    AverageBuffer timeBuffer;

    Timer timer;

    @Override
    public void init() {
        encoder1 = hardwareMap.get(DcMotor.class, "encoder1");
        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2 = hardwareMap.get(DcMotor.class, "encoder2");
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder3 = hardwareMap.get(DcMotor.class, "encoder3");
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new Timer();
        timeBuffer = new AverageBuffer(10);
    }

    @Override
    public void loop() {
        timer.updateTime();

        timeBuffer.update(timer.getDeltaTime());
        double deltaTime = timeBuffer.getValue();

        telemetry.addLine(String.format("Update rate: %.3f Hz", 1.0 / deltaTime));

        log(encoder1);
        log(encoder2);
        log(encoder3);

        if (this.gamepad1.x) {
            reset();
        }
    }

    void reset() {
        encoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        encoder3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void log(DcMotor encoder) {
        telemetry.addLine(String.format("Connection: %s", encoder.getConnectionInfo()));
        telemetry.addLine(String.format("Position: %d", encoder.getCurrentPosition()));
        telemetry.addLine("-------------------------------");
    }

}
