package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Camera.CameraEnums.CameraModes;
import org.firstinspires.ftc.teamcode.Camera.CameraEnums.SpikeMarkPositions;

import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;

@TeleOp
public class CameraTest extends OpMode {
    OpenCvWebcam webcam;

    int[] dimensions = new int[] { 640, 360 };

    TeamPropMask teamPropMask;

    DefaultMask defaultMask = new DefaultMask();
    boolean isUsingDefault;

    ButtonToggle xToggle;
    ButtonToggle yToggle;

    @Override
    public void init() {
        teamPropMask = new TeamPropMask(dimensions[0], dimensions[1], telemetry);
        telemetry.setAutoClear(false);
        xToggle = new ButtonToggle();
        yToggle = new ButtonToggle();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.setPipeline(teamPropMask);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(dimensions[0], dimensions[1], OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error " + errorCode, "error accessing camera stream");
            }
        });
    }
    int width = 640;
    int height = 360;

    Rect leftRect = RectangleFactory.generateRectFromPercentages(width, height, 0, 50, 27, 100);
    Rect rightRect = RectangleFactory.generateRectFromPercentages(width, height, 36, 46, 74, 70);
    Rect centerRect = RectangleFactory.generateRectFromPercentages(width, height, 73, 50, 100, 100);
    @Override
    public void init_loop() {
//        telemetry.addData(">", "Press X to set Default vs mask");
//        telemetry.addData(">", "Press Y to set red or blue when in masked mode");
//        telemetry.addData("Left", String.format(Locale.ENGLISH, "(%d, %d), %d by %d", leftRect.x, leftRect.y, leftRect.width, leftRect.height));
        xToggle.update(gamepad1.x);
        yToggle.update(gamepad1.y);

        if (xToggle.getValue()) {
            if (isUsingDefault) {
                webcam.setPipeline(teamPropMask);
                isUsingDefault = false;
            }
            else {
                webcam.setPipeline(defaultMask);
                isUsingDefault = true;
            }
        }

        if (yToggle.getValue()) {
            if (teamPropMask.getMode().equals("red")) {
                teamPropMask.setMode(CameraModes.BLUE);
            }
            else {
                teamPropMask.setMode(CameraModes.RED);
            }
        }

//        telemetry.addData("Mode", isUsingDefault ? "Default" : teamPropMask.getMode());


    }

    @Override
    public void loop() {


    }


}
