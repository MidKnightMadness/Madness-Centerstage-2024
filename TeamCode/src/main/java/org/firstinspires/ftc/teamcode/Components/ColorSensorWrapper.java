package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Utility.RGBColor;

import java.util.ArrayList;

public class ColorSensorWrapper {
    ColorSensor colorSensor;
    int BUFFER_SIZE = 10;
    ArrayList<RGBColor> rgbBuffer = new ArrayList<RGBColor>();
    RGBColor value = new RGBColor();

    public ArrayList<RGBColor> getRgbBuffer() {
        return rgbBuffer;
    }

    public ColorSensorWrapper(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    public ColorSensorWrapper(ColorSensor colorSensor, int bufferSize) {
        this.colorSensor = colorSensor;
        this.BUFFER_SIZE = bufferSize;
    }

    public void clearBuffer() {
        this.rgbBuffer.clear();
    }

    public void update() {
        rgbBuffer.add(new RGBColor(colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()));

        if (this.BUFFER_SIZE == rgbBuffer.size()) {
            value = RGBColor.average(rgbBuffer);
            rgbBuffer.clear();
        }
    }

    public RGBColor getValue() {
        return value;
    }
}
