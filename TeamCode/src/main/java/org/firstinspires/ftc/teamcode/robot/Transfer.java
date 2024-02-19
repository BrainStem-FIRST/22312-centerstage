package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Transfer {
    private ServoImplEx oneWayGate;
    private Constants constants = new Constants();

    public ElapsedTime transferCycleTime = new ElapsedTime();

    public final String TRANSFER_SYSTEM_NAME = "TRANSFER_SYSTEM_NAME";
    public final String TRANSFER_GATE_CLOSED = "TRANSFER_GATE_CLOSED";
    public final String TRANSFER_GATE_OPEN = "TRANSFER_GATE_OPEN";
    private int gateMinPWM = 1000;
    private int gateMaxPWM = 1800;
    private double gateClosedPosition = 0;
    private double gateOpenPosition = 1;
    private Telemetry telemetry;
    private Map stateMap;

    public Transfer(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        oneWayGate =  hwMap.get(ServoImplEx.class, "transferGate");
        oneWayGate.setPwmRange(new PwmControl.PwmRange(gateMinPWM, gateMaxPWM));
    }

    public void setState(){
        if(cycleIsInProgress()){
            stateMap.put(TRANSFER_SYSTEM_NAME, TRANSFER_GATE_CLOSED);
            if(transferCycleTime.milliseconds() > 250){
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
    private void selectTransition(){
        String desiredState = (String) stateMap.get(TRANSFER_SYSTEM_NAME);

        switch(desiredState){
            case TRANSFER_GATE_CLOSED:{
                setOneWayGatePosition(gateClosedPosition);
                break;
            }

            case TRANSFER_GATE_OPEN:{
                setOneWayGatePosition(gateOpenPosition);
                break;
            }
        }
    }

    private void setOneWayGatePosition(double position){
        oneWayGate.setPosition(position);
    }
}