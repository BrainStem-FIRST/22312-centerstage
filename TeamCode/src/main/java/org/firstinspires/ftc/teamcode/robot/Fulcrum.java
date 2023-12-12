package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Fulcrum {
    private Telemetry telemetry;
    private Map stateMap;
    private ServoImplEx fulcrumServo;
    private final double down = 1.0;
    private final double up = 0;
    private double lowerPWMLimit = 620;
    private double upperPWMLimit = 1390;
    public ElapsedTime fulcrumCycleDownTime = new ElapsedTime();
    public ElapsedTime fulcrumCycleUpTime = new ElapsedTime();
    private Constants constants = new Constants();

    //Statemap strings
    public final String FULCRUM_SYSTEM_NAME = "FULCRUM_SYSTEM_NAME";
    public final String FULCRUM_DOWN = "FULCRUM_DOWN";
    public final String FULCRUM_UP = "FULCRUM_UP";
    public Fulcrum(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap =  stateMap;
        fulcrumServo = hwMap.get(ServoImplEx.class, "fulcrumServo");

        fulcrumServo.setPwmRange(new PwmControl.PwmRange(lowerPWMLimit,upperPWMLimit));

    }

    public void setState(Lift lift){
        telemetry.addData("Fulcrum cycle in progress", cycleIsInProgress());
        if(cycleIsInProgress()){
            String fulcrumCycleDownState = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN);
            String fulcrumCycleUpState = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP);
            if(fulcrumCycleDownState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
                stateMap.put(FULCRUM_SYSTEM_NAME, FULCRUM_DOWN);
                if(fulcrumCycleDownTime.milliseconds() > 500){
                    stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN, constants.PIXEL_CYCLE_STATE_COMPLETE);
                }
            } else if(fulcrumCycleUpState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
                stateMap.put(FULCRUM_SYSTEM_NAME, FULCRUM_UP);
                if(fulcrumCycleUpTime.milliseconds() > 400){
                    stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP, constants.PIXEL_CYCLE_STATE_COMPLETE);
                }
            }
        }
        selectTransition(lift);
    }

    private boolean cycleIsInProgress(){
        String fulcrumCycleDownState = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN);
        String fulcrumCycleUpState = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP);
        if(fulcrumCycleDownState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS) || fulcrumCycleUpState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            return true;
        }
        return false;
    }
    private void selectTransition(Lift lift){
        String desiredState = (String) stateMap.get(FULCRUM_SYSTEM_NAME);
        switch(desiredState){
            case FULCRUM_DOWN:{
                if(lift.liftMotor1.getCurrentPosition() < 100){
                    fulcrumUp();
                } else{
                    fulcrumDown();
                }
                break;
            }
            case FULCRUM_UP:{
                fulcrumUp();
                break;
            }
        }
    }

    private void fulcrumDown(){
        fulcrumServo.setPosition(down);
    }

    private void fulcrumUp(){
        fulcrumServo.setPosition(up);
    }
}
