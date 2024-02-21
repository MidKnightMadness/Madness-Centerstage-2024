package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Camera.TeamPropMask;
import org.firstinspires.ftc.teamcode.Components.LinearSlides;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/* For testing opmodes without odometry
Objectives:
1. Driving forward by distance, whether by wheel radii calculations or by calibrated time and power calculations
2. Smooithly ramp up and ramp down power
Contains:
1. Drive using power and time (primarily used)
2. Drive using distance
3. Conversion functions and compensation for RPM-torque differences between motors
 */
@TeleOp
@SuppressLint("DefaultLocale")
public class AutoDeadReckoning extends OpMode implements WheelRPMConfig, ServoPositions {
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
    Thread thread;
    public StartingPosition getStartingPosition() {
        return StartingPosition.NEAR;
    }
    public double slidesExtensionTimeConstant = 1.9;
    public double rammingPower = 0.6;

    CameraModes cameraMode = getAllianceColor();
    public DeadReckoningDrive deadReckoningDrive;
    IMU imu;
    SpikeMarkPositions teamPropPosition = SpikeMarkPositions.LEFT;
    Servo intakeRightServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    ModernRoboticsI2cRangeSensor rangeSensor;
    Timer timer;
    ButtonToggle a, b, x, y;
    OpenCvWebcam webcam;
    public WebcamName webcamName;
    TeamPropMask teamPropMask;
    LinearSlides slides;
    ServoSmooth boxServoController;

    // Testing for April tag incorporation
    AprilTagProcessor aprilTagProcessor;

    @Override
    public void init() {
        timer = new Timer();
        thread = new Thread();
        slides = new LinearSlides(hardwareMap);
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");
        boxServo = hardwareMap.get(Servo.class, "Center box servo");
        boxServoController = new ServoSmooth(boxServo);
        init_IMU();

        teamPropMask = new TeamPropMask(640, 360, telemetry);
        teamPropMask.setMode(getAllianceColor());
        intakeRightServo = hardwareMap.get(Servo.class, "Right intake servo");
        intakeRightServo.setPosition(0.1);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Front Distance Sensor");

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        webcam.setPipeline(teamPropMask);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error " + errorCode, "error accessing camera stream");
            }
        });
    }

    @Override
    public void init_loop() {
        telemetry.clear();
        teamPropPosition = teamPropMask.getSpikeMarkPosition();
        telemetry.addData("imu yaw", deadReckoningDrive.getRobotDegrees());
        telemetry.addData("Detected spike mark position", teamPropPosition);
        deadReckoningDrive.updateDisplacement();
        telemetry.addData("Displacement", deadReckoningDrive.getDisplacement());
        if(cameraMode == CameraModes.RED){
            telemetry.addLine("Camera Mode: RED");
        }else{
            telemetry.addLine("Camera Mode: BLUE");
        }
    }

    @Override
    public void start() {
        imu.resetYaw();
        webcam.stopStreaming();

        // Reset servos
        boxServo.setPosition(boxServoNeutral);
        rightWristServo.setPosition(wristServoIn);
        intakeRightServo.setPosition(intakeHighest);

        drive();
    }

    @Override
    public void stop(){
        telemetry.addLine("STOPPED");
        telemetry.update();
    }

    void sleep(long milis) {
        try {
            Thread.sleep(milis);
        }
        catch (InterruptedException e) {
            telemetry.addData("Error", e.getMessage());
        }

    }
    double increment = 0.0005;
    double kp = 0.2;

    @Override
    public void loop() {
        telemetry.clear();

        if (y.update(gamepad1.y)) {
//            deadReckoningDrive.moveRightDistance(-24);
            deadReckoningDrive.setTargetRotation(0);
//            setTargetRotation(Math.round(getRobotDegrees()) + 90);
//            driveForwardForTime(2, 0.5);
        }
//
        if (a.update(gamepad1.a)) {
            deadReckoningDrive.setTargetRotation(180);
//            setTargetRotation(Math.round(getRobotDegrees()) - 90);
//            driveForward(12);
        }
//
        if (b.update(gamepad1.b)) {
            deadReckoningDrive.setTargetRotation(90);
        }
        if (x.update(gamepad1.x)) {
            deadReckoningDrive.setTargetRotation(-90);
        }
//
//        if (gamepad1.left_bumper) {
//            setPowers(1, 1, 1, 1);
//        }
//        else {
//            setPowers(0, 0, 0, 0);
//        }
//
        if (this.gamepad1.dpad_up) {
            kp += increment;
        }

        if (gamepad1.dpad_down) {
            kp -= increment;
        }

        telemetry.addData("Robot degrees", deadReckoningDrive.getRobotDegrees());
    }

    void init_IMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    double [] motorInputs = {0.0, 0.0, 0.0, 0.0};
    double previousX = 0.0;
    double previousY = 0.0;
    public static final double [] FORWARD = {-1.0, 1.0, -1.0, -1.0};
    ///forward: -1.0, 1.0, -1.0, -1.0
    public static final double [] RIGHT = {-1.0, -1.0, 1.0, -1.0};
    public static final double [] CLOCKWISE = {-1.0, -1.0, -1.0, 1.0};
    public static final double POWER_MULTIPLIER = 1;
    public void FieldOrientedDrive(double x, double y, double rotation, double angle, Telemetry telemetry){ // Angle of front from horizontal right, meant for controller inputs
        double maxPowerLevel = 0.0;

        // Low pass
        double lowPassX = 0.05 * x + 0.95 * previousX;
        double lowPassY = 0.05 * y + 0.95 * previousY;


        // Rotate x and y by negative of angle
        double newX = lowPassX*Math.cos(angle - (Math.PI / 2.0)) + lowPassY*Math.sin(angle - (Math.PI / 2.0));
        double newY = -lowPassX*Math.sin(angle - (Math.PI / 2.0)) + lowPassY*Math.cos(angle - (Math.PI / 2.0));

        // Update low pass previous variables
        previousX = x;
        previousY = y;

        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * newY) + (RIGHT [i] * newX) + (CLOCKWISE [i] * rotation)) * POWER_MULTIPLIER * RPMMultipliers[i] ;

            if(Math.abs(motorInputs [i]) > maxPowerLevel){
                maxPowerLevel = Math.abs(motorInputs [i]);
            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]);
        if(powerEnvelope > 0.2 && maxPowerLevel > 1.0){
            for(int i = 0; i < 4; i++){
                motorInputs [i] /= maxPowerLevel;
            }
        }

//        telemetry.addData("\nPower envelope", powerEnvelope);
//        telemetry.addData("Max Power", maxPowerLevel);
//        telemetry.addData("X", newX);
//        telemetry.addData("Y", newY);
//
        telemetry.addData("FL", motorInputs [0]);
        telemetry.addData("FR", motorInputs [1]);
        telemetry.addData("BL", motorInputs [2]);
        telemetry.addData("BR", motorInputs [3]);
//        telemetry.addData("Low pass latency", 0.5);

        setMotorPowers();
    }

    void setMotorPowers() {
        deadReckoningDrive.FL.setPower( motorInputs [0]);
        deadReckoningDrive.FR.setPower( motorInputs [1]);
        deadReckoningDrive.BL.setPower( motorInputs [2]);
        deadReckoningDrive.BR.setPower( motorInputs [3]);
    }

    double getRobotDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public void drive(){}

    public void rotateBoxTo(double position) {
        double servoPosition = boxServo.getPosition();
        while(boxServo.getPosition() < position){
            boxServo.setPosition(boxServo.getPosition() + 0.01);
            try {
                Thread.sleep((long) Math.round(10.0 * Math.PI / (Math.PI)));
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}

/* One more easter egg!
/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import java.util.List;
        import org.firstinspires.ftc.robotcore.external.ClassFactory;
        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
        import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
        import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.


//@Disabled
public class AutoVisual {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model_20220309_001537.tflite";
    private static final String[] LABELS = {
            "Element"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.

    private static final String VUFORIA_KEY =
            "ARHVqML/////AAABmdwV3COoyUo6te5Z9nV9Xbs58R8vE55rTErE0ztbuXhfaoos0oD/3ZcFBeJ+b0gLISGqWdDOBM9m4cv6rMlzbJ2qLTB9KX5EpbWfKO2fo9LUIYHLWbre2dui4BfhgLuvKxP8nT/yBsEjAUVz61Bzf3gIEFPTaF8jAnVLwUmYO2Y7/8bXyCNTCoYnC74qHS9D0mqbg+LlGVelz4Zg3zpFfIgwYi56uvaTpdVAxYmPao5JQ0h9FJYLuvfPs9znZEU6QNkS83GVoRm5/cd4S52lWr1jcoeFWg2Haqn7wxKfFGgS7fB41O9wxOe/FHO5Yz4RV0jfYp7M97PxUOvE+c6tOipsSIJnL0aZwYHPRBbX48jA";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.

    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.

    private TFObjectDetector tfod;
    int pos = 0;
    //0 == Unknown
    //1 == left
    //2 == middle
    //3. == right
    public AutoVisual(HardwareMap hardwareMap) {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");









        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.94f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);










        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.5, 16.0/5.0);
        }
    }
    /**
     * Activate TensorFlow Object Detection before we wait for the start command.
     * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
     *


    /** Wait for the game to begin

    public int Visual(Telemetry telemetry){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    telemetry.addData(String.format(" confidence: ", i), recognition.getConfidence());
                    i++;
                    detectPosition results = getTeamElementPosition(recognition);
                    if (results == detectPosition.left) {
                        telemetry.addData("Block position: ", "right");
                        pos = 1;
                        break;
                    } else if (results == detectPosition.center) {
                        telemetry.addData("Block position: ", "center");
                        pos = 2;
                        break;
                    } else if (results == detectPosition.left) {
                        telemetry.addData("Block position: ", "left");
                        pos = 3;
                        break;

                    }
                }
                telemetry.update();
            }
        }
        return pos;
    }


    /**
     * Initialize the Vuforia localization engine.

    //private void initVuforia() {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    // parameters.vuforiaLicenseKey = VUFORIA_KEY;
    // parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

    // //  Instantiate the Vuforia engine
    // vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    //}

    /**
     * Initialize the TensorFlow Object Detection engine.

    //  private void initTfod() {
    //     int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
    //         "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //   TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
    //   tfodParameters.minResultConfidence = 0.8f;
    //   tfodParameters.isModelTensorFlow2 = true;
    //   tfodParameters.inputSize = 320;
    //   tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    //   tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    // }

    private enum detectPosition {
        left, right, center;
    }

    public detectPosition getTeamElementPosition(Recognition recognition) {
        double avgWidth = (Math.abs(recognition.getRight() - recognition.getLeft()) / 2);
        double imgWidth = recognition.getImageWidth();
        if (avgWidth < (recognition.getImageWidth() / 3)) {
            pos = 1;
            return detectPosition.left;
            //return pos;
        } else if ((avgWidth > (recognition.getImageWidth() / 3)) && (avgWidth < (int) (recognition.getImageWidth() * (2 / 3)))) {
            pos = 2;
            return detectPosition.center;
            //return pos;
        } else if (avgWidth > (int) (recognition.getImageWidth() * (2 / 3))) {
            pos = 3;
            return detectPosition.right;
            //return pos;
        } else {
            return null;
        }

    }

    public int returnPosition(){
        return pos;
    }

}

 */