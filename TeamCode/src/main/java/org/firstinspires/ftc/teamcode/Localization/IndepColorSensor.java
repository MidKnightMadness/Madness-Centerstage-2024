package org.firstinspires.ftc.teamcode.Localization;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IndepColorSensor implements Runnable{
    public ColorSensor colorSensor;

    public IndepColorSensor (HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
    }
    @Override
    public void run() {

    }
}
