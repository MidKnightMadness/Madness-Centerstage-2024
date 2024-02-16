package org.firstinspires.ftc.teamcode.Camera;

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
import org.firstinspires.ftc.robotcore.internal.tfod.TFObjectDetectorImpl;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;

public class TeamPropMask2 implements FrameGenerator{
    protected final TfodParameters parameters;
    protected final TFObjectDetector tfObjectDetector;
    protected final Object frameConsumerLock = new Object();
    protected FrameConsumer frameConsumer;
    protected Bitmap bitmap;
    protected int width;
    protected int height;
    protected float fx, fy = 100; // dummy


    public TeamPropMask2(TfodParameters parameters) {
        this.parameters = parameters;
        tfObjectDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, this);
    }

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

    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        FrameConsumer frameConsumerSafe;
        Bitmap bitmapSafe;

        synchronized (frameConsumerLock)
        {
            if (frameConsumer == null)
            {
                return null;
            }
            frameConsumerSafe = frameConsumer;
            bitmapSafe = bitmap;
        }

        Utils.matToBitmap(frame, bitmapSafe);
        return frameConsumerSafe.processFrame();
    }

    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {
        if (userContext != null)
        {
            ((CanvasAnnotator) userContext).draw(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
        }
    }

    public CameraInformation getCameraInformation()
    {
        return new CameraInformation(width, height, 0, fx, fy);
    }

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

    public void setMinResultConfidence(float minResultConfidence) {
        tfObjectDetector.setMinResultConfidence(minResultConfidence);
    }

    public void setClippingMargins(int left, int top, int right, int bottom)
    {
        tfObjectDetector.setClippingMargins(left, top, right, bottom);
    }

    public void setZoom(double magnification)
    {
        tfObjectDetector.setZoom(magnification);
    }

    public List<Recognition> getRecognitions()
    {
        return tfObjectDetector.getRecognitions();
    }

    public List<Recognition> getFreshRecognitions()
    {
        return tfObjectDetector.getUpdatedRecognitions();
    }

    public void shutdown()
    {
        tfObjectDetector.shutdown();
    }
}
