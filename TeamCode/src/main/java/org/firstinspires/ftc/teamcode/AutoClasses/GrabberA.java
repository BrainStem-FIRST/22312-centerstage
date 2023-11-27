package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class GrabberA {

    private double grabberOpenPosition;
    private double grabberPick1PixelPosition;
    private double grabberPick2PixelPosition;
    private double grabberDeposit1Pixel;
    private double grabberDeposit2Pixel;

    private Telemetry telemetry;

    private ServoImplEx grabber;

    private int grabberPWMHigherlimit;
    private int grabberPWMLowerLimit;

    public GrabberA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        grabber = hwMap.get(ServoImplEx.class, "grabber");
        grabber.setPwmRange(new PwmControl.PwmRange(grabberPWMLowerLimit, grabberPWMHigherlimit));
    }

    public void setPosition (double position){
        grabber.setPosition(position);
    }
}

