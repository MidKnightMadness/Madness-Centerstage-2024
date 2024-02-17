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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
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
public class ConceptDoubleVisionForTeamProp extends LinearOpMode {
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

    @Override
    public void runOpMode() {
        initDoubleVision();

        // This OpMode loops continuously, allowing the user to switch between
        // AprilTag and TensorFlow Object Detection (TFOD) image processors.
        while (!isStopRequested())  {

            if (opModeInInit()) {
                telemetry.addData("DS preview on/off","3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
            }

            if (myVisionPortal.getProcessorEnabled(aprilTag)) {
                // User instructions: Dpad left or Dpad right.
                telemetry.addLine("Dpad Left to disable AprilTag");
                telemetry.addLine();
                telemetryAprilTag();
            } else {
                telemetry.addLine("Dpad Right to enable AprilTag");
            }
            telemetry.addLine();
            telemetry.addLine("----------------------------------------");
            if (myVisionPortal.getProcessorEnabled(tfod)) {
                telemetry.addLine("Dpad Down to disable TFOD");
                telemetry.addLine();
                telemetryTfod();
            } else {
                telemetry.addLine("Dpad Up to enable TFOD");
            }

            // Push telemetry to the Driver Station.
            telemetry.update();

            if (gamepad1.dpad_left) {
                myVisionPortal.setProcessorEnabled(aprilTag, false);
            } else if (gamepad1.dpad_right) {
                myVisionPortal.setProcessorEnabled(aprilTag, true);
            }
            if (gamepad1.dpad_down) {
                myVisionPortal.setProcessorEnabled(tfod, false);
            } else if (gamepad1.dpad_up) {
                myVisionPortal.setProcessorEnabled(tfod, true);
            }

            sleep(20);

        }   // end while loop

    }   // end method runOpMode()


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

                telemetry.clear();


                Imgproc.rectangle(output, leftRect, leftColor, 4);
                Imgproc.rectangle(output, rightRect, rightColor, 4);
                Imgproc.rectangle(output, centerRect, centerColor, 4);

                return output;
            }

            @Override
            public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

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

        for(AprilTagDetection detection : currentDetections) { // Calculate range coefficient
            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);

            if(detection.id == 3 || detection.id == 4) {
                rangeCoefficient += 1d /
                        (detection.ftcPose.range * detection.ftcPose.range);
            }else {
                rangeCoefficient += 0.1 /
                        (detection.ftcPose.range * detection.ftcPose.range);
            }
        }

        for(AprilTagDetection detection : currentDetections) {
            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);

            if(detection.id == 3 || detection.id == 4) {
                heading += (detection.ftcPose.yaw * 1d) /
                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else if(detection.id > 6) {
                heading += ((detection.ftcPose.yaw + Math.PI) * 0.1) /
                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else {
                heading += (detection.ftcPose.yaw * 0.1) /
                        (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }
        }

        for(AprilTagDetection detection : currentDetections) {
            double actualRange = correctAprilTagError(detection.id, detection.ftcPose.range);

            if(detection.id == 3 || detection.id == 4) {
                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 1d / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));

                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][1]) * 1d / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }else {
                cameraCoordinates [0] -= (Math.cos(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][0]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));

                cameraCoordinates [1] -= (Math.sin(heading + detection.ftcPose.bearing) * detection.ftcPose.range
                        -APRIL_TAG_COORDS [detection.id  - 1][1]) * 0.1 / (rangeCoefficient * (detection.ftcPose.range * detection.ftcPose.range));
            }
        }

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

}   // end class
