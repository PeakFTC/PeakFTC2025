package org.firstinspires.ftc.teamcode.pedroPathing.PeakFTC2025.FTC_TeamCode.source;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class colorSensorTest extends OpMode {
    private colorSensorPeak colorSensor;
    @Override
    public void init(){
        colorSensor = new colorSensorPeak(hardwareMap, telemetry);
        colorSensor.initColorSensorPeak();

    }
    @Override
    public void  start(){

    }
    @Override
    public void loop(){
        telemetry.addData("is Empty",colorSensor.isOuttakeEmpty());
        telemetry.update();
    }
}
