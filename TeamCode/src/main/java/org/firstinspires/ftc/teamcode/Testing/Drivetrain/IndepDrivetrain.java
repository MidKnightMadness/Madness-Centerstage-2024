package org.firstinspires.ftc.teamcode.Testing.Drivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Testing.SplineDrive.SectionSpline;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

public class IndepDrivetrain implements Runnable{
    public MecanumDrive mecanumDrive;
    public Odometry odometry;
    public org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive PIDDrive;
    public SectionSpline spline;
    public Telemetry telemetry;
    public ElapsedTime timer;

// AUXILLARY VARIABLES =============================================================================
    public double lastTime = 0.0;
    public double updateRate = 0.0;
    public int operationalMode = 0; // 0 is field oriented drive, 1 is point to point, 2 is spline
    public boolean active = false;
    double [] controllerInputs = {0.0, 0.0, 0.0};
    double [] PIDInputs = {0.0, 0.0, 0.0};


//  SETTINGS =======================================================================================
    // For non-spline-path driving - Madness robot 2022
    double [] P = {0.4, 0.4, 0.0};
    double [] I = {0.1, 0.1, 0.0};
    double [] D = {0.05, 0.06, 0.0};
    double [] D2 = {0.0, 0.0, 0.0};

    // For spline path driving - Madness robot 2022
    double [] PSpline = {0.4, 0.4, 0.0};
    double [] ISpline = {0.1, 0.1, 0.0};
    double [] DSpline = {0.0, 0.0, 0.0};
    double [] D2Spline = {0.0, 0.0, 0.0};

    public IndepDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, double x0, double y0, double startingAngle){
        this.telemetry = telemetry;

        this.mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        this.odometry = new Odometry(hardwareMap, startingAngle, new Vector2(x0, y0));

        timer = new ElapsedTime();
        operationalMode = 0;
    }

    public IndepDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, double x0, double y0, double startingAngle, double targetX0, double targetY0, double targetAngle0){
        this.telemetry = telemetry;

        this.mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        this.odometry = new Odometry(hardwareMap, startingAngle, new Vector2(x0, y0));
        this.PIDDrive = new PIDDrive(odometry, targetX0, targetY0, targetAngle0, telemetry);

        this.PIDDrive.P = this.P;
        this.PIDDrive.I = this.I;
        this.PIDDrive.D = this.D;
        this.PIDDrive.D2 = this.D2;

        timer = new ElapsedTime();
        operationalMode = 1;
    }

    public IndepDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, double x0, double y0, double startingAngle, double [][] roots, double weight){
        this.telemetry = telemetry;

        this.mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        this.odometry = new Odometry(hardwareMap, startingAngle, new Vector2(x0, y0));
        this.PIDDrive = new PIDDrive(odometry, x0, y0, startingAngle, telemetry);

        this.PIDDrive.P = this.PSpline;
        this.PIDDrive.I = this.ISpline;
        this.PIDDrive.D = this.DSpline;
        this.PIDDrive.D2 = this.D2Spline;

        this.spline = new SectionSpline(roots, weight);

        timer = new ElapsedTime();
        operationalMode = 2;
    }

    @Override
    public void run() { // Might help with update rate
        timer.startTime();
        active = true;

        while(active){
            // Updates
            odometry.updatePosition();
            updateRate = 1000.0 / (timer.milliseconds() - lastTime);
            telemetry();

            switch(operationalMode){
                case 0: // FIELD ORIENTED DRIVE ====================================================
                    mecanumDrive.FieldOrientedDrive(controllerInputs [0], controllerInputs [1], controllerInputs [2], odometry.getRotationRadians(), telemetry);
                    break;

                case 1: // POINT TO POINT PID ======================================================
                    PIDInputs = PIDDrive.updatePID();
                    mecanumDrive.FieldOrientedDrive(PIDInputs [0], PIDInputs [1], PIDInputs [2], odometry.getRotationRadians(), telemetry);
                    break;

                case 2: // SPLINE ==================================================================
                    PIDInputs = PIDDrive.updatePID();
                    mecanumDrive.FieldOrientedDrive(PIDInputs [0], PIDInputs [1], PIDInputs [2], odometry.getRotationRadians(), telemetry);
                    break;
            }

            if(timer.milliseconds() != 0){
                lastTime = timer.milliseconds();
            }
        }
    }

    public void toggle(){
        active = !active;
    }

    public void telemetry(){
        telemetry.addData("x", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());

        telemetry.addData("\nleft encoder", odometry.leftEncoder.getCurrentPosition());
        telemetry.addData("right encoder", odometry.rightEncoder.getCurrentPosition());
        telemetry.addData("center encoder", odometry.horizontalEncoder.getCurrentPosition());
        telemetry.update();
    }
}
