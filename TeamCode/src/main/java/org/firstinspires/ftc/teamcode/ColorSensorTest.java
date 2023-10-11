package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ColorSensorTest extends OpMode {

    ColorSensor cs;


    @Override
    public void init() {
        // Init
        cs = hardwareMap.get(ColorSensor.class, "test color sensor");


    }

    @Override
    public void loop() {
        //color sensor is from 1 inch away
        if (cs.red() >= 430 && cs.red() <= 550 && cs.green() >= 860 && cs.green() <= 1000 && cs.blue() >= 920&& cs.blue() <= 1100) {
            //maximum and minimum for white color
            telemetry.addLine("Color: White");
        }
        else if(cs.red()>=60&&cs.red()<=120&&cs.green()>=150&&cs.green()<=240&&cs.blue()>=440&&cs.blue()<=550){
            telemetry.addLine("Color: Blue");
        }
        else if(cs.red()>=150&&cs.red()<=260&&cs.green()>=120&&cs.green()<=220&&cs.blue()>=70&&cs.blue()<=170){
            telemetry.addLine("Color: Red");
        }
        else{
            telemetry.addLine("Color: Unidentified");
        }


        telemetry.addLine("RGB Values/nRed: "+cs.red()+"/nGreen:" + cs.green()+"/nBlue:" + cs.blue()+ "/nAlpha:" + cs.alpha());
        telemetry.update();
    }
}
