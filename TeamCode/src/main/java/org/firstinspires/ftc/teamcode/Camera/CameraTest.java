package org.firstinspires.ftc.teamcode.Camera;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class CameraTest extends OpMode {
    OpenCvWebcam webcam;

    int[] dimensions = new int[] {640, 360 };

    TeamPropMask teamPropMask =  new TeamPropMask(dimensions[0], dimensions[1]);

    DefaultMask defaultMask = new DefaultMask();
    boolean isUsingDefault;

    ButtonToggle xToggle;
    ButtonToggle yToggle;

    @Override
    public void init() {
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

    @Override
    public void init_loop() {
        telemetry.addData(">", "Press X to set Default vs mask");
        telemetry.addData(">", "Press Y to set red or blue when in masked mode");
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
                teamPropMask.setMode("blue");
            }
            else {
                teamPropMask.setMode("red");
            }

        }

        telemetry.addData("Mode", isUsingDefault ? "Default" : teamPropMask.getMode());
    }

    @Override
    public void loop() {


    }


}
