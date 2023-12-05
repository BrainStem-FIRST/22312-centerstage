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
    private double grabberDeposit1Pixel;
    private double grabberDeposit2Pixel;

    private Telemetry telemetry;

    public CRServo grabber;

    public GrabberA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        grabber = hwMap.get(CRServo.class, "grabber");
    }

    public void setPower (double power){
        grabber.setPower(power);
        telemetry.addData("grabber power", grabber.getPower());
    }

    public Action grabPixel = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    setPower(0.8);
                    return false;
                }
            }
    );
}

