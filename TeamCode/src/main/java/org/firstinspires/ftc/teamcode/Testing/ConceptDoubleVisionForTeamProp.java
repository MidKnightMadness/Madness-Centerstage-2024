/* Copyright (c) 2023 FIRST. All rights reserved.
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
 */

package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Autonomous.DeadReckoningDrive;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.CameraLocalizationPackage;
import org.firstinspires.ftc.teamcode.Utility.Timer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

/*
 * This OpMode illustrates the basics of using both AprilTag recognition and TensorFlow
 * Object Detection.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: Double Vision with Team Element Detection", group = "Concept")
public class ConceptDoubleVisionForTeamProp extends OpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal myVisionPortal;

    // Variables and objects for team prop detection
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

    int teamPropPosition = 0;

    // For April Tag Localization
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
    double [] CAMERA_RELATIVE_COORDINATES = {0d, 0d};
    double heading = 0d;
    double [] cameraCoordinates = {0d, 0d};
    double [] robotCoordinates = {0d, 0d};
    double rangeCoefficient = 0.0; // Used as average intermediate

    MecanumDrive mecanumDrive;
    DeadReckoningDrive deadReckoningDrive;
    IMU imu;

    // For alignment with camera
    double P = 0.1;
    double D = 0.075;
    double [] perceivedPosition = {0.0, 0.0};
    double [] targetCoordinates = {118.5, 109d};
    double [] deltaPosition = {0.0, 0.0};
    double [] lastPosition = {0.0, 0.0};
    double [] velocity = {0.0, 0.0};
    final double targetAngle = 0;

    @Override
    public void init(){
        initDoubleVision();

        init_IMU();
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);
        imu.resetYaw();


        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    @Override
    public void start() {
            deadReckoningDrive.moveRightDistance(aprilTag.getDetections().get(0).ftcPose.x);

            telemetry.addLine(String.format("Target: [%5.2f, %5.2f]", targetCoordinates [0], targetCoordinates [1]));
            telemetry.addLine(String.format("Current Position: [%5.2f, %5.2f]", perceivedPosition [0], perceivedPosition [1]));
            telemetry.addLine(String.format("Delta: [%5.2f, %5.2f]", aprilTag.getDetections().get(0).ftcPose.x, 0));// deltaPosition [1]));
            telemetry.addData("Angle correction", 0.2 * (targetAngle - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            telemetry.update();



    }   // end method runOpMode()

    @Override
    public void loop() {

    }

    @Override
    public void init_loop(){
        perceivedPosition = getRelCoords(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), perceivedPosition [0], perceivedPosition [1]);
        deltaPosition [0] = targetCoordinates [0] - perceivedPosition [0];
        deltaPosition [1] = targetCoordinates [1] - perceivedPosition [1];
        velocity [0] = perceivedPosition [0] - lastPosition [0];
        velocity [1] = perceivedPosition [1] - lastPosition [1];

        telemetry.addLine(String.format("Delta: [%5.2f, %5.2f]", deltaPosition [0], deltaPosition [1]));
        telemetry.update();

        telemetryAprilTag();
    }

    /**
     * Initialize AprilTag and TFOD.
     */
    private void initDoubleVision() {
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

                if (mode == CameraEnums.CameraModes.RED) {
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
                    teamPropPosition = 0;
                }
                else if (right > left && right > center) {
                    rightColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.RIGHT;
                    teamPropPosition = 1;
                }
                else {
                    centerColor = detectedRectColor;
                    position = CameraEnums.SpikeMarkPositions.CENTER;
                    teamPropPosition = 2;
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
    }   // end initDoubleVision()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {
        telemetry.addData("Detected spike mark position", teamPropPosition);
    }   // end method telemetryTfod()

    // Localization Method
    public double [] getRelCoords(double robotHeading, double currentX, double currentY){ // Camera coordinates, relative to field; x, y, heading
        // Reset calculation variables
        rangeCoefficient = 0d;
        heading = 0d;
        cameraCoordinates [0] = 0d;
        cameraCoordinates [1] = 0d;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(currentDetections.size() == 0 || currentDetections == null){ // Works for all tags!
            return new double [] {currentX, currentY, robotHeading};
        }

        for(AprilTagDetection detection : currentDetections){
            rangeCoefficient += detection.ftcPose.range;
        }
        rangeCoefficient /= currentDetections.size();

        for(AprilTagDetection detection : currentDetections) {
            heading += detection.ftcPose.yaw;
        }
        heading /= currentDetections.size();

        for(AprilTagDetection detection : currentDetections) {
            cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                    -APRIL_TAG_COORDS [detection.id  - 1][0]) * 1d / currentDetections.size();
            cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                    -APRIL_TAG_COORDS [detection.id  - 1][1]) * 1d / currentDetections.size();

        }

//        for(AprilTagDetection detection : currentDetections) { // Calculate range coefficient
//            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);
//
//            if(detection.id == 3 || detection.id == 4) {
//                rangeCoefficient += 1d /
//                        (detection.ftcPose.range * detection.ftcPose.range);
//            }else {
//                rangeCoefficient += 0.1 /
//                        (detection.ftcPose.range * detection.ftcPose.range);
//            }
//        }
//
//        for(AprilTagDetection detection : currentDetections) {
//            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);
//
//            if(detection.id == 3 || detection.id == 4) {
//                heading += (detection.ftcPose.yaw * 1d) /
//                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//            }else if(detection.id > 6) {
//                heading += ((detection.ftcPose.yaw + Math.PI) * 0.1) /
//                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//            }else {
//                heading += (detection.ftcPose.yaw * 0.1) /
//                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//            }
//        }
//
//        for(AprilTagDetection detection : currentDetections) {
//            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);
//
//            if(detection.id == 3 || detection.id == 4) {
//                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
//                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 1d / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//
//                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
//                        -APRIL_TAG_COORDS [detection.id  - 1][1]) * 1d / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//            }else {
//                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
//                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//
//                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
//                        -APRIL_TAG_COORDS [detection.id  - 1][1]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
//            }
//        }

        return cameraCoordinates;
    }


    private double correctY(double percievedY){
        return (percievedY + 1.200) / 1.068;
    }

    private double correctX(double percievedX, double actualY){
        return (percievedX - (actualY * 0.06898) + 0.2844) / (-1.0425);
    }

    private double [][] TAG_RANGE_CORRECTIONS = { // A, B, C
            // Perceived range = A * exp(C * range) + B
            {2.352 * 10000d, -2.351 * 10000d, 4.179 / 100000d}, // ID 1
            {6503d, -6502d, 0.0001502}, // ID 2
            {3909d, -3908d, 0.0002501}, // ID 3
            {298.8, -297.2, 0.003194}, // ID 4
            {-1.282 * 10000d, 1.282 * 10000d, -7.845 / 100000d}, // ID 5
            {-837.2, 837.6, -0.001248}, // ID 6
    };

    public double correctAprilTagError(int id, double perceivedRange){
        return (2.303 * (1 / Math.log(2.71828182846)) * Math.log((perceivedRange - TAG_RANGE_CORRECTIONS[id - 1][1]) / TAG_RANGE_CORRECTIONS[id - 1][0]) / TAG_RANGE_CORRECTIONS[id - 1][2]);
    }

    void init_IMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }
}   // end class

/* This is an easter egg
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class Daddy extends LinearOpMode{
    DcMotor Daddy1;
    DcMotor Daddy2;
    DcMotor Mommy1;
    DcMotor Mommy2;

    public void runOpMode(){
        Daddy1 = hardwareMap.get(DcMotor.class, "fl");
        Daddy1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Daddy2 = hardwareMap.get(DcMotor.class, "fr");
        Daddy2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Mommy1 = hardwareMap.get(DcMotor.class, "bl");
        Mommy1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Mommy2 = hardwareMap.get(DcMotor.class, "br");
        Mommy2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if(opModeIsActive()){
            while(opModeIsActive()){
                double forwardPower = gamepad1.left_stick_y;
                Daddy1.setPower(forwardPower);
                Daddy2.setPower(-forwardPower);

                double turnPower = gamepad1.right_stick_x;
                if(turnPower!=0){
                Mommy1.setPower(turnPower);
                Mommy2.setPower(turnPower);
                }
                else{
                Mommy1.setPower(-forwardPower);
                Mommy2.setPower(forwardPower);
                }

                if(forwardPower==0){
                    Daddy1.setPower(-gamepad1.left_trigger);
                    Daddy2.setPower(gamepad1.right_trigger);
                }


            }
        }
    }
}
 */