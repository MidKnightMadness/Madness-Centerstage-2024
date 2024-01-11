package org.firstinspires.ftc.teamcode.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Work in progress, meant to quickly tune PID
Objectives:
1. Count number of oscillations and adjust until stabilized
2. Detect when oscillations are dampened to restart tuning cycle
3. Detect when oscillations are going out of control
4. Simulate resetting position or getting pushed
5. Detect critical damping and get optimal D coefficient
6. Get optimal I coefficient (not going too high)
Contains:
1.
 */
@TeleOp(name = "PID Tuner \"auton\"")
public class PIDTuner extends LinearOpMode { // Currently only for x and y corrections
    // HARDWARE ====================================================================================
    MecanumDrive drive;
    Odometry odometry;
//    PIDDrive PIDDrive;

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
