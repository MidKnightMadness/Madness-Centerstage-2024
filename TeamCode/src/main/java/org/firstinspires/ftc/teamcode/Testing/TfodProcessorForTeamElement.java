package org.firstinspires.ftc.teamcode.Testing;
/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.CameraInformation;
import org.firstinspires.ftc.robotcore.external.tfod.CanvasAnnotator;
import org.firstinspires.ftc.robotcore.external.tfod.FrameConsumer;
import org.firstinspires.ftc.robotcore.external.tfod.FrameGenerator;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodParameters;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

class TfodProcessorForTeamElement extends TfodProcessor implements FrameGenerator
{
    protected final TfodParameters parameters;
    protected final TFObjectDetector tfObjectDetector;
    protected final Object frameConsumerLock = new Object();
    protected FrameConsumer frameConsumer;
    protected Bitmap bitmap;
    protected int width;
    protected int height;
    protected float fx, fy = 100; // dummy
    Scalar defaultRectColor = new Scalar(255, 255, 255); // white
    Scalar detectedRectColor = new Scalar(100, 150, 255); // gray
    CameraEnums.SpikeMarkPositions position = CameraEnums.SpikeMarkPositions.LEFT;
    int teamPropPosition = 0;

    public TfodProcessorForTeamElement(TfodParameters parameters)
    {
        this.parameters = parameters;
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, this);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        this.width = width;
        this.height = height;

        if (calibration != null)
        {
            fx = calibration.focalLengthX;
            fy = calibration.focalLengthY;
        }

        tfObjectDetector.activate();
    }

    // Stuff for Team Prop Mask
    Mat hsvMat = new Mat();
    Mat output = new Mat();
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
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        Mat redOutput1 = new Mat();
        Mat redOutput2 = new Mat();

        // red bounds
        Core.inRange(hsvMat, redLower, redUpper, redOutput1);
        Core.inRange(hsvMat, redLower2, redUpper2, redOutput2);
        Core.bitwise_or(redOutput1, redOutput2, output);

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
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        if (userContext != null)
        {
            ((CanvasAnnotator) userContext).draw(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
        }
    }

    @Override
    public CameraInformation getCameraInformation()
    {
        return new CameraInformation(width, height, 0, fx, fy);
    }

    @Override
    public void setFrameConsumer(FrameConsumer frameConsumer)
    {
        synchronized (frameConsumerLock)
        {
            this.frameConsumer = frameConsumer;

            if (frameConsumer != null)
            {
                bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
                frameConsumer.init(bitmap);
            }
        }
    }

    @Override
    public void setMinResultConfidence(float minResultConfidence) {
        tfObjectDetector.setMinResultConfidence(minResultConfidence);
    }

    @Override
    public void setClippingMargins(int left, int top, int right, int bottom)
    {
        tfObjectDetector.setClippingMargins(left, top, right, bottom);
    }

    @Override
    public void setZoom(double magnification)
    {
        tfObjectDetector.setZoom(magnification);
    }

    @Override
    public List<Recognition> getRecognitions()
    {
        return tfObjectDetector.getRecognitions();
    }

    @Override
    public List<Recognition> getFreshRecognitions()
    {
        return tfObjectDetector.getUpdatedRecognitions();
    }

    @Override
    public void shutdown()
    {
        tfObjectDetector.shutdown();
    }
}
