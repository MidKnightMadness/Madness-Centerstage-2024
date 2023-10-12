package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;

@TeleOp
public class ColorSensorTest extends OpMode {

    ColorSensor cs;
    ColorSensorWrapper csw;

    public final int RedMin[] = {430, 60,150};

    public final int RedMax[] = {550,120,260};
    public final int GreenMin[] = {860, 150,120};
    public final int GreenMax[] = {1000,240, 220};
    public final int BlueMin[] = {920, 440,70};
    public final int BlueMax[] = {1100,550,170};
    @Override
    public void init() {
        // Init
        cs = hardwareMap.get(ColorSensor.class, "test color sensor");
        csw = new ColorSensorWrapper(cs);
    }

    @Override
    public void loop() {
        //color sensor is from 1 inch away
        csw.update();

        for(int i=0;i<3;i++){
            if(cs.red()>=RedMin[i]&&cs.red()<=RedMax[i]&&cs.green()>=GreenMin[i]&&cs.green()<=GreenMax[i]&&cs.blue()>=BlueMin[i]&&cs.blue()<=BlueMax[i]){
                if(i==0){
                    telemetry.addLine("Color: White");
                }
                if(i==1){
                    telemetry.addLine("Color: Blue");
                }
                if(i==2){
                    telemetry.addLine("Color: Red");
                }
            }
        }

        telemetry.addLine("RGB Values: " + csw.getValue());
        telemetry.update();
    }
}
