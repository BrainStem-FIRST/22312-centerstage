package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Wrist {
    ServoImplEx wristServo;

    private int minPWM = 760;
    private int maxPWM = 1771;

    public String WRIST_SYSTEM_NAME = "WRIST_SYSTEM_NAME";
    public final String WRIST_0_DEGREE_STATE = "WRIST_0_DEGREE_STATE";
    public final String WRIST_45_DEGREE_STATE = "WRIST_45_DEGREE_STATE";
    public final String WRIST_90_DEGREE_STATE = "WRIST_90_DEGREE_STATE";
    public final String WRIST_135_DEGREE_STATE = "WRIST_135_DEGREE_STATE";
    public final String WRIST_180_DEGREE_STATE = "WRIST_180_DEGREE_STATE";

    private Map stateMap;
    private Telemetry telemetry;

    private double zero_degree_position = 0.0;
    private double fourty_five_degree_position = 0.25;
    private double ninety_degree_position = 0.5;
    private double one_thirty_five_degree_position = 0.75;
    private double one_eighty_degree_position = 1.0;

    public Wrist(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        wristServo = hwMap.get(ServoImplEx.class, "wristServo");
        wristServo.setPwmRange(new PwmControl.PwmRange(minPWM, maxPWM));
    }

    public void setState(){
        selectTransition();
    }

    private void selectTransition(){
        String desiredState = (String) stateMap.get(WRIST_SYSTEM_NAME);
        switch(desiredState){
            case WRIST_0_DEGREE_STATE:{
                wristSetPosition(zero_degree_position);
                break;
            }
            case WRIST_45_DEGREE_STATE:{
                wristSetPosition(fourty_five_degree_position);
                break;
            }
            case WRIST_90_DEGREE_STATE:{
                wristSetPosition(ninety_degree_position);
                break;
            }
            case WRIST_135_DEGREE_STATE:{
                wristSetPosition(one_thirty_five_degree_position);
                break;
            }
            case WRIST_180_DEGREE_STATE:{
                wristSetPosition(one_eighty_degree_position);
                break;
            }
        }
    }

    private void wristSetPosition(double position){
        wristServo.setPosition(position);
    }
}
