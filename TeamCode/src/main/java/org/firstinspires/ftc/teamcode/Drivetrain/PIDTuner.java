package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "PID Tuner \"auton\"")
public class PIDTuner extends LinearOpMode { // Currently only for x and y corrections
    // HARDWARE ====================================================================================
    MecanumDrive drive;
    Odometry odometry;
    PIDDrive PIDDrive;

    // NUMBERS AND CALCULATIONS ====================================================================
    double [][] PID = { // Initial variables
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}
    };

    @Override
    public void runOpMode() throws InterruptedException {
        // INIT LOOP ===============================================================================

        // P TUNING ================================================================================
    }

    public double tuneP(){
        return 0.0;
    }

    public double tuneD(){
        return 0.0;
    }

    public double tuneI(){
        return 0.0;
    }
}
