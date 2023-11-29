package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Camera.PixelDetector;
import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.SectionSpline;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

@TeleOp(group= "Test", name = "Basic driving test")
public class DrivingTest extends OpMode {
    // HARDWARE ====================================================================================
    MecanumDrive mecanumDrive;
    Odometry odometry;

    // PROCESSING ==================================================================================
    PIDDrive PIDDrive;
    SectionSpline spline;

    double [] driveInputs = {0.0, 0.0, 0.0};
    double [][] targetStates = {

    };


    @Override
    public void init() {
        // HARDWARE INIT ===========================================================================
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0d, new Vector2(0.0, 0.0));
        odometry.resetEncoders();
        odometry.setRotation(Math.PI / 2.0d);

        // OTHER INIT ==============================================================================
        PIDDrive = new PIDDrive(odometry, 50.0, 1.0, Math.PI / 2.0d, telemetry);
//        PIDDrive.setTargetState(0.0, 20.0, Math.PI / 2.0d);
    }

    @Override
    public void loop() {
        // DON'T CHANGE ============================================================================
//        if(gamepad1.x){
//            odometry.resetEncoders();
//        }

        // NORMAL DRIVER CONTROL ===================================================================
//        mecanumDrive.normalDrive(1, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

        // PID TUNING ==============================================================================
            // PID Adjustment
            if(gamepad1.dpad_down){
                PIDDrive.D [0] -= 0.001;
                PIDDrive.D [1] -= 0.001;
            }else if(gamepad1.dpad_up){
                PIDDrive.D [0] += 0.001;
                PIDDrive.D [1] += 0.001;
            }

            PIDDrive.P [0] -= 0.001 * gamepad1.left_stick_y;
            PIDDrive.P [1] -= 0.001 * gamepad1.left_stick_y;

            PIDDrive.I [0] -= 0.001 * gamepad1.right_stick_y;
            PIDDrive.I [1] -= 0.001 * gamepad1.right_stick_y;

            telemetry.addData("D", PIDDrive.D [0]);
            telemetry.addData("P", PIDDrive.P [0]);
            telemetry.addData("I", PIDDrive.I [0]);

            odometry.updatePosition();
            driveInputs = PIDDrive.updatePID();
        if(PIDDrive.distanceToTarget > 0.5) {
            mecanumDrive.FieldOrientedDrive(-driveInputs[0], -driveInputs[1], -driveInputs[2], odometry.getRotationRadians(), telemetry);
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

