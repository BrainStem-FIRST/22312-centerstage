package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Depositer {
    private ServoImplEx greenDepositerServo;
    private ServoImplEx redDepositerServo;
    private Telemetry telemetry;
    private Map stateMap;
    private Constants constants = new Constants();
    public ElapsedTime depositerCycleTime = new ElapsedTime();
    public final String DEPOSITER_SYSTEM_NAME = "DEPOSITER_SYSTEM_NAME";
    public final String DEPOSITER_PICKUP_TWO = "DEPOSITER_PICKUP_TWO";
    public final String DEPOSITER_PICKUP_ONE = "DEPOSITER_PICKUP_ONE";
    public final String DEPOSITER_OPEN = "DEPOISTER_OPEN";
    public final String GREEN_DEPOSITER_OPEN = "GREEN_DEPOSITER_OPEN";
    public final String RED_DEPOSITER_OPEN = "RED_DEPOSITER_OPEN";


    private int greenDepositerPWMMin = 1819;
    private int greenDepositerPWMMax = 2500;

    private int redDepositerPWMMin = 1378;
    private int redDepositerPWMMax = 2200;

    public Depositer(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        greenDepositerServo = hwMap.get(ServoImplEx.class, "greenDepositerServo");
        redDepositerServo = hwMap.get(ServoImplEx.class, "redDepositerServo");

        greenDepositerServo.setPwmRange(new PwmControl.PwmRange(greenDepositerPWMMin, greenDepositerPWMMax));
        redDepositerServo.setPwmRange(new PwmControl.PwmRange(redDepositerPWMMin, redDepositerPWMMax));

    }

    public void setState() {
        if(cycleIsInProgress()){
            stateMap.put(DEPOSITER_SYSTEM_NAME, DEPOSITER_PICKUP_TWO);
            if(depositerCycleTime.milliseconds() > 250){
                stateMap.put(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE, constants.PIXEL_CYCLE_STATE_COMPLETE);
            }
        }
        selectTransition();
    }
    private boolean cycleIsInProgress(){
        String depositerCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE);
        if(depositerCycleState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            return true;
        }
        return false;
    }
    private void selectTransition() {
        String desiredState = (String) stateMap.get(DEPOSITER_SYSTEM_NAME);
        switch (desiredState) {
            case DEPOSITER_PICKUP_TWO: {
                setGreenServoPosition(1.0);
                setRedServoPosition(1.0);
                break;
            }
            case DEPOSITER_PICKUP_ONE: {
                setGreenServoPosition(1.0);
                setRedServoPosition(0);
                break;
            }
            case DEPOSITER_OPEN: {
                setGreenServoPosition(0);
                setRedServoPosition(0);
                break;
            }
            case GREEN_DEPOSITER_OPEN:{
                setGreenServoPosition(0);
                break;
            }
            case RED_DEPOSITER_OPEN:{
                setRedServoPosition(0);
                break;
            }
        }
    }

    private void setRedServoPosition(double position){
        redDepositerServo.setPosition(position);
    }

    private void setGreenServoPosition(double position){
        greenDepositerServo.setPosition(position);
    }
}

