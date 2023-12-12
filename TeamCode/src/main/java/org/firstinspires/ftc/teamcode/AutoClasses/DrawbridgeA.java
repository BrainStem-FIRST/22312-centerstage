package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class DrawbridgeA {

    private final int drawbridgePWMLowerLimit = 850;
    private final int drawbridgePWMHigherLimit = 1600;

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
        drawBridge.setPosition(1);
    }
    public void setDrawBridgeDown(){
        telemetry.addData("Drawbridge State", "down");
        drawBridge.setPosition(0);
    }
}
