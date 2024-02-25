package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class WristA {

    ServoImplEx wristServo;

    private int minPWM = 750;
    private int maxPWM = 1748;

    private Telemetry telemetry;

    private double zero_degree_position = 0.0;
    private double fourty_five_degree_position = 0.25;
    private double ninety_degree_position = 0.48;
    private double one_thirty_five_degree_position = 0.75;
    private double one_eighty_degree_position = 1.0;

    public WristA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        wristServo = hwMap.get(ServoImplEx.class, "wristServo");
        wristServo.setPwmRange(new PwmControl.PwmRange(minPWM, maxPWM));
    }

    public void wristToPickUpPosition(){
        wristServo.setPosition(ninety_degree_position);
    }

    public void wristToZeroDegreePosition(){
        wristServo.setPosition(zero_degree_position);
    }
}
