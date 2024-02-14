package org.firstinspires.ftc.teamcode.Testing.ColorSensor;

//implement this class into Auto/TeleOp

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.RGBColor;
import org.firstinspires.ftc.teamcode.Testing.ColorSensor.PixelEnum.pixelColors;

public class OutakeColorSensors {//implementable file(takes care of color sensor hardware map implementation)
    //methods to use in main/auto
    ColorSensor outerColorSensor;
    ColorSensor innerColorSensor;
    Telemetry telemetry;
    public OutakeColorSensors(HardwareMap hardwareMap, Telemetry telemetry){
        innerColorSensor = hardwareMap.get(ColorSensor.class, "Outake inner color sensor");
        outerColorSensor = hardwareMap.get(ColorSensor.class, "Outake outer color sensor");
        this.telemetry = telemetry;
    }
    double[][] colors = {{0.9, 0.9, 0.9},//white
            {0.6, 0.1, 0.9},//purple
            {0.6, 0.8, 0.2},//yellow
            {0.55, 0.9, 0.55} //green
    };
    double leniency = 0.05;//changable

    int pixelCount = 0;

    PixelEnum.pixelColors pixelColorInner;
    PixelEnum.pixelColors pixelColorOuter;
    public void updateTelemetry(){
            pixelCount = 0;
            pixelColorInner = checkColor(innerColorSensor);
            pixelColorOuter = checkColor(outerColorSensor);
            if (pixelColorInner != PixelEnum.pixelColors.None) {
                pixelCount++;
            }
            if (pixelColorOuter != PixelEnum.pixelColors.None) {
                pixelCount++;
            }

        telemetry.addData("Inner pixel spot ", pixelColorInner);
        telemetry.addData("Outer pixel spot ", pixelColorOuter);
        telemetry.addData("Number of pixels ", pixelCount);

    }

    public pixelColors checkColor(ColorSensor colorSensor){//checks the color of the pixles
        RGBColor rgbColor = new RGBColor(colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha());
        rgbColor.normalizeRGB();
        pixelColors pixelColor = PixelEnum.pixelColors.None;
        for(int i = 0;i<4;i++){//each of the four pixels that the color sensor detection could be
            if((rgbColor.r > (colors[i][0] -leniency) && rgbColor.r < (colors[i][0] + leniency)) &&
                    (rgbColor.g > (colors[i][1] -leniency) && rgbColor.g < (colors[i][1] + leniency)) &&
                    (rgbColor.b > (colors[i][2] -leniency) && rgbColor.b < (colors[i][2] + leniency))){

                pixelColor = i==0 ? pixelColors.White : i==1 ?  pixelColors.Purple : i ==2 ? pixelColors.Yellow
                        : pixelColors.Green;
                //setting the pixel color
                return pixelColor;
            }
        }
        return pixelColor;//if no color
    }
}
