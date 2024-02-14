package org.firstinspires.ftc.teamcode.Testing.ColorSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Testing.ColorSensor.PixelEnum.pixelColors;
import org.firstinspires.ftc.teamcode.Utility.RGBColor;


//assuming color sensors are 1/4-1/2 inch away from pixels
@TeleOp(name = "Color Sensor Testing")
public class OutakeColorSensorsTesting extends OpMode {

    ColorSensor outerColorSensor;
    ColorSensor innerColorSensor;

    //variables with the magnitude percentages
    //arbitrary values need to be changed(more testing required)

    //R, G, B values, in order
    double[][] colors = {{0.9, 0.9, 0.9},//white
                        {0.6, 0.1, 0.9},//purple
                        {0.6, 0.8, 0.2},//yellow
                        {0.55, 0.9, 0.55} //green
                        };
    double leniency = 0.05;//amount of +- space for each color
    @Override
    public void init() {
        innerColorSensor = hardwareMap.get(ColorSensor.class, "Outake inner color sensor");
        outerColorSensor = hardwareMap.get(ColorSensor.class, "Outake outer color sensor");
    }

    int pixelCount = 0;
    pixelColors pixelColorInner;
    pixelColors pixelColorOuter;
    @Override
    public void loop() {
        telemetry.clear();
       if(gamepad1.x) {
           pixelCount = 0;
           pixelColorInner = checkColor(innerColorSensor);
           pixelColorOuter = checkColor(outerColorSensor);
           if (pixelColorInner != pixelColors.None) {
               pixelCount++;
           }
           if (pixelColorOuter != pixelColors.None) {
               pixelCount++;
           }
       }
           telemetry.addData("Inner pixel spot ", pixelColorInner);
           telemetry.addData("Outer pixel spot ", pixelColorOuter);
           telemetry.addData("Number of pixels ", pixelCount);

    }

    public pixelColors checkColor(ColorSensor colorSensor){//checks the color of the pixles
        RGBColor rgbColor = new RGBColor(colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha());
        rgbColor.normalizeRGB();
        pixelColors pixelColor = pixelColors.None;
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
