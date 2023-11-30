package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Camera.PixelDetector;
import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Drivetrain.PIDDrive;
import org.firstinspires.ftc.teamcode.Drivetrain.SectionSpline;
import org.firstinspires.ftc.teamcode.Drivetrain.SplinePath;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;

@TeleOp(group= "[Game]", name = "Driver Controlled TeleOp")
public class Main extends OpMode {

    public ColorSensorWrapper colorSensorWrapper;
    public PixelDetector pixelDetector;

    MecanumDrive mecanumDrive;

    ColorSensor colorSensorOutake;

    ColorSensor colorSensorChasis;

    ButtonToggle buttonToggleA;

    Servo servoBox;
    Servo armIntake;
    DcMotorEx IntakeMotor;

    public int numberOfTimesATrue = 0;

    public boolean down;
    public boolean up;

    //just using driver controlled
    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);

        //initiate servo intake motor
//         IntakeMotor = hardwareMap.get(DcMotorEx.class, "servoIntake");
//         IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//         IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        //initialization of color sensors
//          colorSensorOutake = hardwareMap.get(ColorSensor.class, "colorSensorOutake");
//          colorSensorChasis = hardwareMap.get(ColorSensor.class, "colorSensorChasis");
//
//
//          servoBox = hardwareMap.get(Servo.class, "servoBox");
//          armIntake = hardwareMap.get(Servo.class,"armIntakeServo");

//       //initialization of wrappers
//        colorSensorWrapper = new ColorSensorWrapper(colorSensorOutake);
//        pixelDetector = new PixelDetector();
//
//        //initialize the button togglers
        buttonToggleA = new ButtonToggle();


        telemetry.addLine("Initialized");
    }

    // Uses gamepad1
    public void handleDriverControls() {
        mecanumDrive.normalDrive(1, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }

    // Uses gamepad2
    boolean isIntakeRunning = false;

    //0.25 is starting position of servo
    boolean outakeBoxLeft = false;
    boolean outakeBoxRight = false;
    //float previouslinearSlidesHeight = 0F;//or whatever value linear slides start at

    //float currentLinearSlidesHeight = 0F;//current linear slides height
    public void handleManipulatorControls() {

        //set for gamepad 2, checking to see if gamepad had any changes

        //linear slides -> left stick y     -> separate class
        //intake -> servo -> d pad down and up -> height
        //button two to start roller intake             -> done
        //button x and b for outake box left and right  -> done

        //button a to run intake running to get pixels
        if (isIntakeRunning) {
            IntakeMotor.setPower(1);
        }
        else {
            IntakeMotor.setPower(0);
        }

        if (buttonToggleA.update(gamepad1.a)) {
            isIntakeRunning = !isIntakeRunning;
        }


    }

    @Override
    public void loop() {
        handleDriverControls();
    }




}

