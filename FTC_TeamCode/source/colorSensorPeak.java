package org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.DecodeIntakeTest;


public class colorSensorPeak {
    private NormalizedColorSensor colorSensor1;
    private NormalizedColorSensor colorSensor2;
    private HardwareMap hardwareMap;
    private  Telemetry telemetry;

    public colorSensorPeak(HardwareMap hMap, Telemetry tel){
        hardwareMap=hMap;
        telemetry = tel;
    }
    private enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public void initColorSensorPeak() {
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor2");

    }
    private DetectedColor detectColor() {
        NormalizedRGBA colors = colorSensor1.getNormalizedColors();

        float red = colors.red * 10000;
        float green = colors.green * 10000;
        float blue = colors.blue * 10000;
        telemetry.addData("sen1 Gain=",colorSensor1.getGain());
        telemetry.addData("red", red);
        telemetry.addData("green", green);
        telemetry.addData("Blue", blue);


        if (green > red && green > blue && green > 20) return DetectedColor.GREEN;
        if (red > 10 && blue > 17 && (blue > (green-1))) return DetectedColor.PURPLE;

        return DetectedColor.UNKNOWN;
    }

    private DetectedColor detectColor1() {

        NormalizedRGBA colors1 = colorSensor2.getNormalizedColors();
        telemetry.addData("sen2 Gain=",colorSensor2.getGain());
        float red = colors1.red * 10000;
        float green = colors1.green * 10000;
        float blue = colors1.blue * 10000;
        telemetry.addData("red1", red);
        telemetry.addData("green1", green);
        telemetry.addData("Blue1", blue);

        if (green > red && green > blue && green > 11) return DetectedColor.GREEN;
        if (red > 7 && blue > 10 && blue > green) return DetectedColor.PURPLE;

        return DetectedColor.UNKNOWN;
    }

    public boolean isOuttakeEmpty(){

        DetectedColor color1,color2;
        color1=detectColor();
        color2=detectColor1();
        telemetry.addData("Color1", color1);
        telemetry.addData("Color2", color2);

        if( (color1.name() == DetectedColor.UNKNOWN.name()) &&
                (color2.name() == DetectedColor.UNKNOWN.name())){
            return true;
        }
        return false;
    }

    public String getColorString() {
        return detectColor().name();
    }
}
