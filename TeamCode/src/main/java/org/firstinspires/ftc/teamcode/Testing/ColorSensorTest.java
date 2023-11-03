package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Utility.RGBColor;

@TeleOp
public class ColorSensorTest extends OpMode {

    ColorSensor cs;
    ColorSensorWrapper csw;

    public final double RedMin[] = {0.27, 0.08,0.66};

    public final double RedMax[] = {0.37,0.22,0.85};
    public final double GreenMin[] = {0.60, 0.27,0.42};
    public final double GreenMax[] = {0.70,0.44, 0.59};
    public final double BlueMin[] = {0.65, 0.85,0.30};
    public final double BlueMax[] = {0.75,0.97,0.44};



    @Override
    public void init() {
        // Init
        cs = hardwareMap.get(ColorSensor.class, "color_sensor");
        csw = new ColorSensorWrapper(cs);//creates new color sensor wrapper

    }

    @Override
    public void loop() {
        //color sensor is from 1 inch away
        csw.update();//in the color sensor wrapper
        //when updating the color sensor wrapper, add the values of rgb to the rgbColor,
        RGBColor color = csw.getValue();//passing on the csw.values into rgbcolor color
        boolean condition = false;
        for(int i=0;i<3;i++){
            if(color.r>=RedMin[i] && color.r <= RedMax[i] && color.g >= GreenMin[i] && color.g <=GreenMax[i] && color.b>=BlueMin[i]&&color.b<=BlueMax[i]) {
                if (i == 0) {
                    telemetry.addLine("Color: White");
                    condition = true;
                }
                if (i == 1) {
                    telemetry.addLine("Color: Blue");
                    condition = true;
                }
                if (i == 2) {
                    telemetry.addLine("Color: Red");
                    condition = true;
                }
            }
        }
        //just adding color unidentified if all other colors are false
        if(condition==false){
            telemetry.addLine("Color: Unidentified");
        }

        telemetry.addLine("RGB Values: " + csw.getValue());
        telemetry.update();
    }
}
