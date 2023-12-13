package org.firstinspires.ftc.teamcode.Testing.SplineDrive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

@TeleOp(group= "Test", name = "Spline driving test")
@Disabled
public class SplineDrivingTest extends OpMode {
    // HARDWARE ====================================================================================
    MecanumDrive mecanumDrive;
    Odometry odometry;

    // PROCESSING ==================================================================================
    PIDDrive PIDDrive;
    SectionSpline spline;

    double [] driveInputs = {0.0, 0.0, 0.0};
    double [][] targetStates = {
            {5.0, 0.0, Math.PI / 2.0d},
            {5.0, 27.0, Math.PI},
            {-22.0 - 40.0, 27.0, Math.PI},
            {-22.0 - 40.0, 27.0 + 30.0, Math.PI / 2.0d},
            {5.0, 27.0 + 30.0, Math.PI / 2.0d},
            {5.0, 0.0, Math.PI}
    };

    double NAVIGATIONAL_TOLERANCE = 0.25; // Inches to target
    int numberOfPointsReached = 0;

    @Override
    public void init() {
        // HARDWARE INIT ===========================================================================
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0d, new Vector2(0.0, 0.0));
        odometry.resetEncoders();
        odometry.setRotation(Math.PI / 2.0d);

        // OTHER INIT ==============================================================================
        PIDDrive = new PIDDrive(odometry, targetStates [0][0], targetStates [0][1], targetStates [0][2], telemetry);
//        PIDDrive.setTargetState(0.0, 20.0, Math.PI / 2.0d);
    }

    @Override
    public void loop() {
        // DON'T CHANGE ============================================================================
//        if(gamepad1.x){
//            odometry.resetEncoders();
//        }

        // PID Adjustment
        if(gamepad1.dpad_down){
            PIDDrive.D [2] -= 0.001;
//            PIDDrive.D [1] -= 0.001;
        }else if(gamepad1.dpad_up){
            PIDDrive.D [2] += 0.001;
//            PIDDrive.D [1] += 0.001;
        }

        PIDDrive.P [2] -= 0.001 * gamepad1.left_stick_y;
//        PIDDrive.P [1] -= 0.001 * gamepad1.left_stick_y;

        PIDDrive.I [2] -= 0.001 * gamepad1.right_stick_y;
//        PIDDrive.I [1] -= 0.001 * gamepad1.right_stick_y;

        telemetry.addData("D", PIDDrive.D [2]);
        telemetry.addData("P", PIDDrive.P [2]);
        telemetry.addData("I", PIDDrive.I [2]);

        odometry.updatePosition();
        driveInputs = PIDDrive.updatePID();

        // NORMAL DRIVER CONTROL ===================================================================
//        mecanumDrive.normalDrive(1, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        // PID TUNING ==============================================================================
        mecanumDrive.FieldOrientedDrive(-driveInputs [0], -driveInputs [1], -driveInputs [2], odometry.getRotationRadians(), telemetry);

        if(PIDDrive.distanceToTarget < NAVIGATIONAL_TOLERANCE
                && numberOfPointsReached < targetStates.length - 1
                && Math.abs(odometry.getRotationRadians() - targetStates[numberOfPointsReached][2]) * 180.0d / Math.PI < 2.0d){
            numberOfPointsReached ++;
            PIDDrive.setTargetState(targetStates [numberOfPointsReached][0], targetStates [numberOfPointsReached][1], targetStates [numberOfPointsReached][2]);
        }

        telemetry();
    }

    public void telemetry(){
        telemetry.addData("\nleft ticks", odometry.leftEncoder.getCurrentPosition());
        telemetry.addData("right ticks", odometry.rightEncoder.getCurrentPosition());
        telemetry.addData("center ticks", odometry.horizontalEncoder.getCurrentPosition());

        telemetry.addData("\nx", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("heading", odometry.getRotationDegrees());

        telemetry.update();
    }
}

