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
//        buttonToggleA = new ButtonToggle();


        telemetry.addLine("Initialized");
    }

    // Uses gamepad1
    public void handleDriverControls() {

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

        //linear slides -> left stick y     -> seperate class
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


        //outake box left and right
        if(outakeBoxLeft==true){
            servoBox.setPosition(-1.0);
        }
        else if(outakeBoxRight==true){
            servoBox.setPosition(1.0);
        }

        //each time it is clicked it moves up or down by 0.1
        if(down==true){
            armIntake.setPosition(armIntake.getPosition()-0.1);
        }//0 is going down

        else if(up==true){
            armIntake.setPosition(armIntake.getPosition()+0.1);
        }

        //Intake d pad down and up height



        //update gamepad button A

        if (buttonToggleA.update(gamepad2.a)) {
            isIntakeRunning = !isIntakeRunning;
            //second time button is clicked, it will stop intake motor running
        }



        //outake box left and right
        if(gamepad2.b){
            outakeBoxLeft = true;
        }

        else if(gamepad2.x){
            outakeBoxRight = true;
        }
        // linear slides going to have a seperate classif(gamepad2.left_stick_y!=previouslinearSlidesHeight){
        //     currentLinearSlidesHeight = gamepad2.left_stick_y;
        //}

        if(gamepad2.dpad_down) {
            down = true;
        }

        else if(gamepad2.dpad_up){
            up = true;
        }
    }

    @Override
    public void loop() {
        mecanumDrive.normalDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}

