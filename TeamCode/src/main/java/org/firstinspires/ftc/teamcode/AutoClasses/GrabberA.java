package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class GrabberA {

    private double grabberOpenPosition;
    private double grabberPick1PixelPosition;
    private double grabberPick2PixelPosition;
    private double grabberDeposit1Pixel = 0.6;
    private double grabberGrabPixel = 1.0;
    private double grabberDeposit2Pixel;
    public int grabberPWMLowerLimit = 600;
    public int grabberPWMHigherLimit = 1166;


    private Telemetry telemetry;

    public ServoImplEx grabber;

    public GrabberA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        grabber = hwMap.get(ServoImplEx.class, "grabber");
        grabber.setPwmRange(new PwmControl.PwmRange(grabberPWMLowerLimit, grabberPWMHigherLimit));
    }

    public void grabPixel () {
        grabber.setPosition(grabberGrabPixel);
    }

    public void depositPixel () {
        grabber.setPosition(grabberDeposit1Pixel);
    }

//    public Action grabPixel = new SequentialAction(
//            new Action() {
//                @Override
//                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                    setPower(0.8);
//                    return false;
//                }
//            }
//    );
}

