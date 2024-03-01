package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class DrawbridgeA {

    private final int drawbridgePWMLowerLimit = 500;
    private final int drawbridgePWMHigherLimit = 2500;

    private int hardstopPWMLowerLimit;
    private int hardstopPWMHigherLimit;

    private Telemetry telemetry;

    private ServoImplEx drawBridge;

    private ServoImplEx hardStop;

    private double drawBridgeUpPosition = 1;

    private double drawBridgeDownPosition = 0;

    public DrawbridgeA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        drawBridge = hwMap.get(ServoImplEx.class, "Drawbridge");
        hardStop = hwMap.get(ServoImplEx.class, "intakeHardstop");

        drawBridge.setPwmRange(new PwmControl.PwmRange(drawbridgePWMLowerLimit, drawbridgePWMHigherLimit));
        hardStop.setPwmRange(new PwmControl.PwmRange(hardstopPWMLowerLimit, hardstopPWMHigherLimit));

    }

    public void setDrawBridgeUp(){
        telemetry.addData("Drawbridge State", "up");
        drawBridge.setPosition(0.01);
    }
    public void setDrawBridgeDown(){
        telemetry.addData("Drawbridge State", "down");
        drawBridge.setPosition(0.99);
    }
    public Action drawBridgeUp = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    setDrawBridgeUp();
                    return false;
                }
            },
            new SleepAction(0.5)
    );
    public Action drawBridgeDown = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    setDrawBridgeDown();
                    return false;
                }
            },
            new SleepAction(0.2)
    );
}
