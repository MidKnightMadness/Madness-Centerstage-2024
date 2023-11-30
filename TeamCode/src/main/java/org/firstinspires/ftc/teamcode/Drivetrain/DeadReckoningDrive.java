package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DeadReckoningDrive extends MecanumDrive{
    double WHEEL_RADIUS = 1.5d;
    double [] GEAR_RATIOS = {25d, 16d, 25d, 16d}; // FL, FR, BL, BR
    int [] TICKS_PER_ROTATION = {3600, 3600, 3600, 3600};
    double STRAFING_MULTIPLIER = 1.41;
    double MAX_SPEED = 5.0; // in / s
    double DEFAULT_SPEED = 5.0; // in / s
    ElapsedTime timer;
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
    double lastTime = 0.0;
    double targetAngle = 0.0;

    public DeadReckoningDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        timer = new ElapsedTime();

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        FL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.6, 0.0, 0.06, 0.0));
//        FR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.6, 0.0, 0.06, 0.0));
//        BL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.6, 0.0, 0.06, 0.0));
//        BR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.6, 0.0, 0.06, 0.0));
    }

    public void setTranslationVelocity(double x, double y){
        double maxVelocity = 0.0;

        for(int i = 0; i < 4; i++){
            motorInputs [i] = STRAFING_MULTIPLIER * x * RIGHT [i] + y * FORWARD[i];
            motorInputs [i] /= GEAR_RATIOS [i] * WHEEL_RADIUS;
            if(Math.abs(motorInputs [i]) > maxVelocity){
                maxVelocity = motorInputs [i];
            }
        }

        if(maxVelocity > MAX_SPEED){
            for(int i = 0; i < 4; i++){
                motorInputs [i] /= maxVelocity;
            }
        }

        FL.setVelocity(motorInputs [0], AngleUnit.RADIANS);
        FR.setVelocity(motorInputs [1], AngleUnit.RADIANS);
        BL.setVelocity(motorInputs [2], AngleUnit.RADIANS);
        BR.setVelocity(motorInputs [3], AngleUnit.RADIANS);
    }
}
