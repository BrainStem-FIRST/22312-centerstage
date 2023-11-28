package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Grabber {
    public final String GRABBER_SYSTEM_NAME = "GRABBER_SYSTEM_NAME";
    public final String GRABBER_OPEN_STATE = "GRABBER_OPEN_STATE";
    public final String GRABBER_PICK_1_PIXEL = "GRABBER_PICK_1_PIXEL";
    public final String GRABBER_PICK_2_PIXEL = "GRABBER_PICK_2_PIXEL";
    public final String GRABBER_DEPOSIT_1st_PIXEL = "GRABBER_DEPOSIT_1_PIXEL";
    public final String GRABBER_DEPOSIT_2nd_PIXEL = "GRABBER_DEPOSIT_2_PIXEL";
    public final String GRABBER_IDLE_STATE = "GRABBER_IDLE_STATE";

    public final String GRABBER_MIDDLE_STATE = "GRABBER_MIDDLE_STATE";

    private double grabberOpenPosition;
    private double grabberPick1PixelPosition;
    private double grabberPick2PixelPosition = 1;
    private double grabberDeposit1Pixel = 0.8;
    private double grabberDeposit2Pixel = 0;

    private double grabberMiddlePosition = 0;

    public ElapsedTime grabberCycleDelay = new ElapsedTime();
    private Telemetry telemetry;
    private Map stateMap;

    private ServoImplEx grabber;

    public ElapsedTime grabberCycleTime = new ElapsedTime();
    private int grabberPWMHigherlimit = 2300;
    private int grabberPWMLowerLimit = 1000;

    Constants constants = new Constants();

    public Grabber(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        grabber = hwMap.get(ServoImplEx.class, "grabber");
        grabber.setPwmRange(new PwmControl.PwmRange(grabberPWMLowerLimit, grabberPWMHigherlimit));
    }

    public void setState(){
        telemetry.addData("In grabber set state", stateMap.get(GRABBER_SYSTEM_NAME));
        if(cycleIsInProgress()){
            stateMap.put(GRABBER_SYSTEM_NAME,GRABBER_PICK_2_PIXEL);
            if(grabberCycleTime.milliseconds() > 500){
                stateMap.put(constants.PIXEL_CYCLE_GRABBER, constants.PIXEL_CYCLE_STATE_COMPLETE);
            }
        }
        selectTransition();
    }

    private boolean cycleIsInProgress(){
        String grabberPixelCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_GRABBER);
        if(grabberPixelCycleState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            return true;
        }
        return false;
    }
    private void selectTransition(){
        String desiredState = (String) stateMap.get(GRABBER_SYSTEM_NAME);
        switch (desiredState){
            case GRABBER_OPEN_STATE:{
                setPosition(grabberOpenPosition);
                break;
            }
            case GRABBER_PICK_1_PIXEL:{
                setPosition(grabberPick1PixelPosition);
                break;
            }
            case GRABBER_PICK_2_PIXEL:{
                setPosition(grabberPick2PixelPosition);
                break;
            }
            case GRABBER_DEPOSIT_1st_PIXEL:{
                setPosition(grabberDeposit1Pixel);
                break;
            }
            case GRABBER_DEPOSIT_2nd_PIXEL:{
                setPosition(grabberDeposit2Pixel);
                break;
            }
            case GRABBER_IDLE_STATE:{
                break;
            }
            case GRABBER_MIDDLE_STATE:{
                setPosition(grabberMiddlePosition);
                break;
            }
        }
    }

    private void setPosition(double position){
        grabber.setPosition(position);
    }
}
