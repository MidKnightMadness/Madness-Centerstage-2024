package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;

@TeleOp(name = "AutonomousTest")
public class AutonomousTest extends OpMode {
   // Drivetrain
    MecanumDrive drive;
    Odometry odometry;
    PIDDrive PIDDrive;

    ElapsedTime timer;

    double [][] targetStates = {
            {40.0, 0.0, 0.0},
            {0.0, -38.0, 0.0},
            {120, 0.0, 0.0},
            {0.0, 0.0, 0.0},

            /*

            { , , ,},
            { , , ,},
            { , , ,},
            { , , ,},

             */
    };

    double [] PIDOutputs = {0.0, 0.0, 0.0};

        @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));
        PIDDrive = new PIDDrive(odometry, 0, 0, 0, telemetry);
        timer =  new ElapsedTime();
        timer.startTime();

        PIDDrive.setTargetState(targetStates [numberOfPointsReached][0], targetStates [numberOfPointsReached][1], targetStates [numberOfPointsReached][2]);
    }


    int numberOfPointsReached = 0;

    public void loop() {

        // Update things
        PIDOutputs = PIDDrive.updatePID();
        odometry.updatePosition();
        // drive.FieldOrientedDrive(PIDOutputs [0], PIDOutputs [1], PIDOutputs [2], odometry.getRotationRadians(), telemetry);
        telemetry.addData("Odometry Y", odometry.getYCoordinate());
        telemetry.addData("Odometry X", odometry.getXCoordinate());

        if(PIDDrive.distanceToTarget < 0.1){
            numberOfPointsReached++;
            if(numberOfPointsReached < targetStates.length){
                //Advance towards spike mark tile, spike mark detection with camera
                PIDDrive.setTargetState(targetStates[numberOfPointsReached][0], targetStates[numberOfPointsReached][1],targetStates[numberOfPointsReached][2]);

                telemetry.addLine("Number of Points Reached: " + numberOfPointsReached);
                telemetry.addLine("Target x: " + targetStates[numberOfPointsReached][0]);
                telemetry.addLine("Target y: " + targetStates[numberOfPointsReached][1]);
                telemetry.addLine("Target angle: " + targetStates[numberOfPointsReached][2] * 180 / Math.PI);
                telemetry.addData("FL Power Level", drive.FL.getPower());
                telemetry.addData("FR Power Level", drive.FL.getPower());
                telemetry.addData("BL Power Level", drive.BL.getPower());
                telemetry.addData("BR Power Level", drive.BR.getPower());


                /*

                //Pixel/Team Prop Camera Detection

                //reverse intake

                //Face backdrop and move between trusses at B3 and Move to B5
//                PIDDrive.setTargetState(0, 84,1.5708);

                //motor to lift outtake
                //camera to
                //servo to drop pixel

                //return to pixel stack
                PIDDrive.setTargetState(0, -120,3.14159);

                //cycle starts
                for (int i = 0; i < 6; i++) {
                    //loops 6 times
                    //intake
                    //back to backdrop
                    PIDDrive.setTargetState(0, 120,3.14159);

                    //april tag to align
                    //lifts outtake
                    //servo drops pixel

                    //back to pixel stack
                    PIDDrive.setTargetState(0, -120,3.14159);
                }

                //park in backstage
                PIDDrive.setTargetState(0, 120,3.14159);*/

            }
        }
    }
}
