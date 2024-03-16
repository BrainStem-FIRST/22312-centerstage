package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class DepositorA {
    private ServoImplEx greenDepositerServo;
    private ServoImplEx redDepositerServo;
    private Telemetry telemetry;

    private int greenDepositerPWMMin = 1126; //810
    private int greenDepositerPWMMax = 1734; //2120

    private int redDepositerPWMMin = 875;
    private int redDepositerPWMMax = 1700;
    public DepositorA(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        greenDepositerServo = hwMap.get(ServoImplEx.class, "greenDepositerServo");
        redDepositerServo = hwMap.get(ServoImplEx.class, "redDepositerServo");

        greenDepositerServo.setPwmRange(new PwmControl.PwmRange(greenDepositerPWMMin, greenDepositerPWMMax));
        redDepositerServo.setPwmRange(new PwmControl.PwmRange(redDepositerPWMMin, redDepositerPWMMax));

    }

    public void grabBothPixels() {
        greenDepositerServo.setPosition(1.0);
        redDepositerServo.setPosition(1.0);
    }

    public void redDepositorPixelGrab() {
        redDepositerServo.setPosition(1.0);
        greenDepositerServo.setPosition(0);
    }

    public void greenDepositorGrab(){
        greenDepositerServo.setPosition(1.0);
        redDepositerServo.setPosition(0);
    }

    public void redDepositorDeposit(){
        redDepositerServo.setPosition(0);
        greenDepositerServo.setPosition(1.0);
    }

    public void greenDepositorDeposit(){
        redDepositerServo.setPosition(1.0);
        greenDepositerServo.setPosition(0);
    }

    public void bothDepositorsDeposit(){
        greenDepositerServo.setPosition(0);
        redDepositerServo.setPosition(0);
    }

    public Action bothDepositorsDeposit = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    bothDepositorsDeposit();
                    return false;
                }
            },
            new SleepAction(0.5)
    );

    public Action bothDepositorsPickup = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    grabBothPixels();
                    return false;
                }
            },
            new SleepAction(0.25)
    );

    public Action bothDepositorPickupSafe =  new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    grabBothPixels();
                    return false;
                }
            },
            new SleepAction(3)
    );

}
