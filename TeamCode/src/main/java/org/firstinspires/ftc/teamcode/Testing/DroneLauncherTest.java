package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.DroneLauncher;

@TeleOp(name = "Drone launcher test", group = "testing")
public class DroneLauncherTest extends OpMode {

    DroneLauncher droneLauncher;
    @Override
    public void init() {
        droneLauncher = new DroneLauncher(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            droneLauncher.open();
        }
        else if(gamepad1.y){
            droneLauncher.lock();
        }
//        if(gamepad1.a){
//            droneLauncher.setPosition();
//        }
        telemetry.addData("Position", droneLauncher.getPosition());
    }
}
