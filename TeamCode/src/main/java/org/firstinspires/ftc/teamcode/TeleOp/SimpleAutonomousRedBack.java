package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "RED BACK")
public class SimpleAutonomousRedBack extends LinearOpMode {
    MecanumDrive mecanumDrive;
    Odometry odometry;
    //no pid drive this time PIDDrive pidDrive;
    ElapsedTime timer;


    //no target states double[][] targetStates{

    // }

    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI/2, new Vector2(0,0));
        // pidDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);
        timer = new ElapsedTime();
        timer.startTime();

        waitForStart();

        timer.reset();
        while(timer.time() < 1000){
            odometry.updatePosition();
            mecanumDrive.normalDrive(-0.5, 0, 0.0);
        }
        mecanumDrive.normalDrive(0.0, 0.0, 0.0);
        Thread.sleep(500);


        while(timer.time() < 3500){
            odometry.updatePosition();
            mecanumDrive.normalDrive(0, 0.5, 0);
        }

    }



}
