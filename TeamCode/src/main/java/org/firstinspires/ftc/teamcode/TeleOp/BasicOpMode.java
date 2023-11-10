package org.firstinspires.ftc.teamcode.TeleOp;

//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.SectionSpline;
import org.firstinspires.ftc.teamcode.Localization.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Localization.IndepColorSensor;
import org.firstinspires.ftc.teamcode.Localization.RGBColor;
import org.firstinspires.ftc.teamcode.Utility.Vector2;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/*
Control hub:
Motor Ports:
0. BL, left encoder
1. BR, front encoder
2. FL
3. FR, right encoder
 */

//@Config
@TeleOp(name = "Basic OpMode")
public class BasicOpMode extends OpMode {
    // Dashboard Obj
//    public static FtcDashboard dashboard;
//    TelemetryPacket packet;
//    MultipleTelemetry dataDump;
    ElapsedTime timer;

    // Hardware
    public MecanumDrive drive;
    public Odometry odometry;
//    public PIDDrive pidDrive;
    public double [][] roots = {
            {0.0, 0.0},
            {10.0, 23.0},
            {0.0, 33.0},
            {-23.0, 23.0}
    };

//    public SectionSpline spline;
    public double [] targetState = {45.0, 0.0, Math.PI / 2.0};

    // Auxillary variables (low pass, PID, etc...)
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    public double t = 0.1;


//    IndepColorSensor indepColorSensor;
    ColorSensorWrapper colorSensorWrapper;
    ColorSensor colorSensor;

    public PIDDrive pidDrive;
    public SectionSpline spline;

    @Override
    public void init() {
//        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();
        timer.startTime();

        // Drivetrain
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));
        pidDrive = new PIDDrive(this.odometry, 0.0, 0.0, - Math.PI, telemetry);
        spline = new SectionSpline(roots, 6.0);

        // Aux data
        previousInputs = new double [3]; // left stick x, left stick y, right stick x

        // Dashboard
//        packet = new TelemetryPacket();
//        dataDump = new MultipleTelemetry();

        telemetry.addLine("init");
        telemetry.update();

        //initializing the color sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
//        indepColorSensor = new IndepColorSensor(hardwareMap, telemetry);
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, 7);
//        indepColorSensor.run();
    }


    // Added boolean for resetting encoders since they started weird
    private boolean encodersReset = false;
    boolean odometryRunning = false;
    double [] PIDOutputs = {0.0, 0.0, 0.0};
    @Override
    public void loop() {

//      BASE PROCEDURE =============================================================================
//        if(gamepad1.x){
//            drive.FL.setPower(0.0);
//            drive.FR.setPower(0.0);
//            drive.BL.setPower(0.0);
//            drive.BR.setPower(0.0);
//            try {
//                Thread.sleep(2000);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }

        if(!encodersReset){
            odometry.resetEncoders();
            odometry.rotationRadians = Math.PI / 2.0;
            encodersReset = true;
            timer.reset();
        }

        odometry.updatePosition();

//        POINT TO POINT PID DRIVING ===============================================================
//        pidDrive.setTargetState(this.targetState [0], this.targetState [1], this.targetState [2]);
//        PIDOutputs = pidDrive.updatePID();


//        drive.FieldOrientedDrive(PIDOutputs [0], PIDOutputs [1], PIDOutputs [2], odometry.getRotationRadians(), telemetry);

        // SPLINE DRIVING CODE =====================================================================
//
//        if(!(t <= 0.05 && -gamepad1.left_stick_y < 0.0) && !(t >= 0.95 && -gamepad1.left_stick_y > 0.0)){
//            t -= gamepad1.left_stick_y * 0.01;
//        }
//        telemetry.addData("left stick y", gamepad1.left_stick_y);
//        telemetry.addData("Time t", t);
//        this.targetState [0] = spline.getState(t)[0];
//        this.targetState [1] = spline.getState(t)[1];
//        telemetry.addData("x target", targetState [0]);
//        telemetry.addData("y target", targetState [1]);
//        this.targetState [2] = spline.getTangentAngle(t);
//
//        pidDrive.setTargetState(this.targetState [0], this.targetState [1], this.targetState [2]);
//        PIDOutputs = pidDrive.updatePID();
//        drive.FieldOrientedDrive(PIDOutputs [0], PIDOutputs [1], PIDOutputs [2], odometry.getRotationRadians(), telemetry);
//        telemetry.update();


        // Start running odometry thread as soon as started, no multi threading rn since running into issues
//        if(!odometryRunning){
//            odometry.run();
//            odometryRunning = true;
//        }

        // NORMAL DRIVE CODE =======================================================================

//        if(gamepad1.left_bumper){
//            drive.FieldOrientedDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
//                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
//                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
//                    odometry.getRotationRadians(), telemetry);
//            telemetry.addLine("Field Oriented");
//
//        }else{
//            odometry.updatePosition();
            drive.NormalDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x, telemetry);

            telemetry.addData("x", odometry.getXCoordinate());
            telemetry.addData("y", odometry.getYCoordinate());
            telemetry.addData("angle", odometry.getRotationDegrees());

            telemetry.addData("\nx velocity", odometry.getVelocity().x);
            telemetry.addData("y velocity", odometry.getVelocity().y);

            telemetry.addData("\nleft encoder", odometry.leftEncoder.getCurrentPosition());
            telemetry.addData("right encoder", odometry.rightEncoder.getCurrentPosition());
            telemetry.addData("center encoder", odometry.horizontalEncoder.getCurrentPosition());
//
//        }
//
//
//        previousInputs [0] = gamepad1.left_stick_x;
//        previousInputs [1] = gamepad1.left_stick_y;
//        previousInputs [2] = gamepad1.right_stick_x;

//        COLOR SENSOR TESTING =====================================================================
//
        colorSensorWrapper.update();
        telemetry.addLine("RGB Values: " + colorSensorWrapper.getValue());

        if(0.78< colorSensorWrapper.getValue().b){
            telemetry.addLine("BLUE");
            detections [reverses] = true;
        }else{
            telemetry.addLine("NO BLUE DETECTED");
        }
        telemetry.addData("Power level:", powerLevel);
        telemetry.addData("Reverses", reverses);
        telemetry.addLine("Detections: [" + detections [0] + ", " + detections [1] + ", " + detections [2] + ", " + detections [3] + ", " + detections [4] + ", " + detections [5] + "]");
        telemetry.addLine("Velocities: [" + velocities [0] + "," + velocities [1] + "," + velocities [2] + "," + velocities [3] + "," + velocities [4] + "," + velocities [5] + "]");
        telemetry.addData("Time", timer.milliseconds() / 1000.00);
        telemetry.addData("FPS", 1.0 / odometry.deltaTime);
        telemetry.addLine("Updated2");
        telemetry.update();

        drive.FieldOrientedDrive(0.0, powerLevel, 0.0, odometry.getRotationRadians(), telemetry);
        if(timer.milliseconds() > 2000 || odometry.getYCoordinate() > 50.0){
            timer.reset();

            if(velocities [reverses] == 0.0){
                velocities [reverses] = odometry.getVelocity().y;

                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            if(reverses < 5){
                reverses++;
            }

            powerLevel *= -0.8;
            timer.startTime();
        }
    }

    boolean [] detections = {false, false, false, false, false, false};
    double [] powerLevels = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double [] velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double powerLevel = 0.4;
    int reverses = 0;

    public void telemetry(){
//        telemetryPacket.fieldOverlay().setFill("pink").fillRect(0, 0, 1000, 500);
//        telemetryPacket.put("uwu", 20);
//        dashboard.sendTelemetryPacket(packet);

//        logGamepad(telemetry, gamepad1, "Raw gamepad inputs");

        telemetry.update();
    }

    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) continue;

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }

            // Adding data:
//            telemetry.addData("Left stick x", gamepad.left_stick_x);
        }
    }
}
