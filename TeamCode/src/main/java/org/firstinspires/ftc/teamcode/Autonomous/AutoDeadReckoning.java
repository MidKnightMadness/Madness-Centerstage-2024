package org.firstinspires.ftc.teamcode.Autonomous;

import android.annotation.SuppressLint;
import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.*;
import org.firstinspires.ftc.teamcode.Components.LinearSlides;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.WheelRPMConfig;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

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
    // OpMode Camera Variables
    public CameraEnums.CameraModes getAllianceColor(){
        return CameraEnums.CameraModes.RED;
    }
    Thread thread;
    CameraModes cameraMode = getAllianceColor();
    public DeadReckoningDrive deadReckoningDrive;
    int [] teamPropPosition = {0, 0, 0}; // Counts up detected in each position

    // Hardware and hardware-related variables
    IMU imu;
    Servo intakeRightServo, leftIntakeServo, boxServo, rightElbowServo, rightWristServo;
    public double slidesExtensionTimeConstant = 1.9;
    public double rammingPower = 0.6;
    ModernRoboticsI2cRangeSensor rangeSensor;
    LinearSlides slides;
    ServoSmooth boxServoController;

    // Auxillary variables
    Timer timer;
    ButtonToggle a, b, x, y;

    // April Tag Detection, correction constants
    private double [][] TAG_RANGE_CORRECTIONS = { // A, B, C
            // Perceived range = A * exp(C * range) + B
            {2.352 * 10000d, -2.351 * 10000d, 4.179 / 100000d}, // ID 1
            {6503d, -6502d, 0.0001502}, // ID 2
            {3909d, -3908d, 0.0002501}, // ID 3
            {298.8, -297.2, 0.003194}, // ID 4
            {-1.282 * 10000d, 1.282 * 10000d, -7.845 / 100000d}, // ID 5
            {-837.2, 837.6, -0.001248}, // ID 6
    };

    public final double[][] APRIL_TAG_COORDS = { // hardcoded
            {135d, 115d},//id 1
            {135d, 109d},//id 2
            {135d, 103d},//id 3
            {135d, 41d},//id 4
            {135d, 35d},//id 5
            {135d, 29d},//id 6
            {0d, 114d},//id 7 not necesarily accurate yet
            {0d, 108d},//id 8 not necesarily accurate yet
            {0d, 36d},//id 9 not necesarily accurate yet
            {0d, 30d}//id 10 not necesarily accurate y
    };

    // Processors and Portals
    boolean USE_WEBCAM = true;
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;
    public VisionPortal myVisionPortal;

    // April Tag Tracking Variables
    double heading = 0d;
    double [] cameraCoordinates = {0d, 0d};
    double rangeCoefficient = 0.0; // Used as average intermediate
    double [] perceivedPosition = {0.0, 0.0};
    double [] targetCoordinates = {118.5, 109d};
    double [] deltaPosition = {0.0, 0.0};
    double [] lastPosition = {0.0, 0.0};
    double [] velocity = {0.0, 0.0};

    // Team Prop Detection Variables
    Mat hsvMat = new Mat();
    Mat output = new Mat();

    CameraEnums.CameraModes mode = CameraEnums.CameraModes.RED;
    CameraEnums.SpikeMarkPositions position = CameraEnums.SpikeMarkPositions.LEFT;

    // blue color bounds
    Scalar blueLower = new Scalar(85, 90, 90);
    Scalar blueUpper = new Scalar(145, 255, 255);

    // red color bounds
    Scalar redLower = new Scalar(0, 100, 100);
    Scalar redUpper = new Scalar(15, 255, 255);
    Scalar redLower2 = new Scalar(160, 100, 100);
    Scalar redUpper2 = new Scalar(180, 255, 255);

    Rect leftRect = new Rect(90, 200, 95, 75);
    Rect rightRect = new Rect(510, 200, 95, 75);
    Rect centerRect = new Rect(300, 185, 95, 75);

    public double[][] coordinates = {
            {1,0,24,42},
            {1,1,36,48},
            {1,2,48,42},

            {2,0,72,42},
            {2,1,84,48},
            {2,2,96,42},

            {3,0,24,102},
            {3,1,36,96},
            {3,2,48,102},

            {4,0,72,102},
            {4,1,84,96},
            {4,2,96,102}
    };

    Scalar defaultRectColor = new Scalar(255, 255, 255); // white
    Scalar detectedRectColor = new Scalar(100, 150, 255); // gray

    @Override
    public void init() {
        // Auxillary Init
        timer = new Timer();
        thread = new Thread();
        slides = new LinearSlides(hardwareMap);
        telemetry.setAutoClear(false);
        a = new ButtonToggle();
        b = new ButtonToggle();
        y = new ButtonToggle();
        x = new ButtonToggle();

        // Init Hardware Elements
        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");
        boxServo = hardwareMap.get(Servo.class, "Center box servo");
        boxServoController = new ServoSmooth(boxServo);
        init_IMU();

        intakeRightServo = hardwareMap.get(Servo.class, "Right intake servo");
        intakeRightServo.setPosition(0.1);
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Front Distance Sensor");

        // Init Vision Portal
        initDoubleVision();
    }

    @Override
    public void init_loop() {
        telemetry.clear();
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

    // Stuff for April Tag Localization
    public void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor() {
            @Override
            public void init(int width, int height, CameraCalibration calibration) {

            }

            @Override
            public Object processFrame(Mat input, long captureTimeNanos) {
                Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

                if (getAllianceColor() == CameraEnums.CameraModes.RED) {
                    Mat redOutput1 = new Mat();
                    Mat redOutput2 = new Mat();

                    // red bounds
                    Core.inRange(hsvMat, redLower, redUpper, redOutput1);
                    Core.inRange(hsvMat, redLower2, redUpper2, redOutput2);
                    Core.bitwise_or(redOutput1, redOutput2, output);
                }
                else {
                    Core.inRange(hsvMat, blueLower, blueUpper, output);
                }

                Scalar leftColor = defaultRectColor;
                Scalar rightColor  = defaultRectColor;
                Scalar centerColor = defaultRectColor;


                Mat leftRectMat = output.submat(leftRect);
                Mat rightRectMat = output.submat(rightRect);
                Mat centerRectMat = output.submat(centerRect);

                Scalar leftAvg = Core.mean(leftRectMat);
                Scalar rightAvg = Core.mean(rightRectMat);
                Scalar centerAvg = Core.mean(centerRectMat);

                double left = leftAvg.val[0];
                double right = rightAvg.val[0];
                double center = centerAvg.val[0];

                if (left > right && left > center) {
                    leftColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.LEFT;
                    teamPropPosition [0]++; // Left
                }
                else if (right > left && right > center) {
                    rightColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.RIGHT;
                    teamPropPosition [1]++; // Right
                }
                else {
                    centerColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.CENTER;
                    teamPropPosition [2]++; // Center
                }


                Imgproc.rectangle(output, leftRect, leftColor, 4);
                Imgproc.rectangle(output, rightRect, rightColor, 4);
                Imgproc.rectangle(output, centerRect, centerColor, 4);

                return output;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
                canvas.drawCircle(320, 180, 50, new Paint());
            }

            @Override
            public void setMinResultConfidence(float minResultConfidence) {

            }

            @Override
            public void setClippingMargins(int left, int top, int right, int bottom) {

            }

            @Override
            public void setZoom(double magnification) {

            }

            @Override
            public List<Recognition> getRecognitions() {
                return null;
            }

            @Override
            public List<Recognition> getFreshRecognitions() {
                return null;
            }

            @Override
            public void shutdown() {

            }
        };

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            myVisionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }

    public SpikeMarkPositions getSpikeMarkPosition(){
        if(teamPropPosition [0] > teamPropPosition [1] && teamPropPosition [0] > teamPropPosition [2] ){ // Left
            return SpikeMarkPositions.LEFT;
        }else if(teamPropPosition [1] > teamPropPosition [0] && teamPropPosition [1] > teamPropPosition [2] ){ // Right
            return SpikeMarkPositions.RIGHT;
        }else{ // Center
            return SpikeMarkPositions.CENTER;
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