package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Components.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Utility.RGBColor;

import java.util.concurrent.TimeUnit;

public class IndepColorSensor implements Runnable{
    public ColorSensor colorSensor;
    public ColorSensorWrapper colorSensorWrapper;

    private final double regulatedFrameInterval = 20; // Milliseconds, currently at 50 hertz
    public boolean active = false;

    public Telemetry telemetry;
    private ElapsedTime timer;

    RGBColor color;

    // Simplified color checking procedure
    public final double RedMin = 0.6;

    public final double RedMax = 1.0;
    public final double GreenMin = 0.27;
    public final double GreenMax = 0.44;
    public final double BlueMin = 0.65;
    public final double BlueMax = 0.75;
    public boolean red = false;
    public boolean green = false;
    public boolean blue = false;

    public IndepColorSensor (HardwareMap hardwareMap, Telemetry telemetry){
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, 2);
        active = true;
        this.telemetry = telemetry;
        timer = new ElapsedTime();
    }

    @Override
    public void run() {
        while(active){ // Meant to be stopped using opmode termination
            colorSensorWrapper.update();

            if(timer.time(TimeUnit.MILLISECONDS) > regulatedFrameInterval){
                timer.reset();

                //color sensor is from 1 inch away
                //in the color sensor wrapper
                //when updating the color sensor wrapper, add the values of rgb to the rgbColor,
                // color = colorSensorWrapper.getValue();//passing on the csw.values into rgbcolor color
                boolean condition = false;

                if(color.r>=RedMin && color.r <= RedMax) {// && color.g >= GreenMin[i] && color.g <=GreenMax[i] && color.b>=BlueMin[i]&&color.b<=BlueMax[i]) {
                    telemetry.addLine("Color: Red");
                    this.red = true;
                }else{
                    this.red = false;
                }

                if(color.g>=GreenMin && color.g <= GreenMax) {// && color.g >= GreenMin[i] && color.g <=GreenMax[i] && color.b>=BlueMin[i]&&color.b<=BlueMax[i]) {
                    telemetry.addLine("Color: Green");
                    this.green = true;
                }else{
                    this.green = false;
                }

                if(color.b>=BlueMin && color.b <= BlueMax) {// && color.g >= GreenMin[i] && color.g <=GreenMax[i] && color.b>=BlueMin[i]&&color.b<=BlueMax[i]) {
                    telemetry.addLine("Color: Blue");
                    this.blue = true;
                }else{
                    this.blue = false;
                }
            }
        }
    }

    public void toggle(){
        active = !active;
    }
}
