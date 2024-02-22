package org.firstinspires.ftc.teamcode.Testing.GryoSensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Utility.Timer;

@TeleOp(name = "Gyro Testing", group = "testing")
public class GryoSensorTesting extends OpMode {
    ModernRoboticsI2cGyro gyro;
    Timer timer;

    @Override
    public void init() {
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro.calibrate();
        gyro.initialize();
    }

    @Override
    public void loop() {
        telemetry.addData("Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle);
//        telemetry.addLine("XYZ" +  gyroSensor.rawX() + " " +  gyroSensor.rawY() + " " +  gyroSensor.rawZ());

//        telemetry.addData("Values" + gyro.rawX() + " " + gyro.rawY() + " "  + gyro.rawZ());
        telemetry.addData("Angular velocity", gyro.getAngularVelocity(AngleUnit.DEGREES));
    }


}
