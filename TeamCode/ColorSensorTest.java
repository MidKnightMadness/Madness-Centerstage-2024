import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorTest extends OpMode {

    ColorSensor cs;


    @Override
    public void init() {
        // Init
        cs = hardwareMap.get(ColorSensor.class, "test color sensor");
        cs.resetDeviceConfigurationForOpMode();

    }

    @Override
    public void loop() {
        //color sensor is from 1 inch away
        if (cs.red() >= 420 && cs.red() <= 440 && cs.green() > 800 && cs.green() < 825 && cs.blue() > 860 && cs.blue() < 890 && cs.alpha() > 690 && cs.alpha() < 720) {
            //maximum and minimum for white color
            telemetry.addLine("Color: White");
        }
        else if(cs.red()>=86&&cs.red()<=88&&cs.green()<=188&&cs.green()>=194&&cs.blue()>=390&&cs.blue()>=405&&cs.alpha()>=220&&cs.alpha()<=235){
            telemetry.addLine("Color: Red");
        }
        else if(cs.red()>=195&&cs.red()<=210&&cs.green()<155&&cs.green()>175&&cs.blue()<110&&cs.blue()>125&&cs.alpha()>155&&cs.alpha()>170){
            telemetry.addLine("Color: Blue")

        }
    }
}