package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Drone {

    public final String DRONE_SYSTEM_NAME = "DRONE_SYSTEM_NAME";
    public final String DRONE_NOT_RELEASED = "DRONE_NOT_RELEASED";
    public final String DRONE_RELEASED = "DRONE_RELEASED";
    private Telemetry telemetry;
    private Map stateMap;
    private ServoImplEx droneServo;
    private int lowerPWM = 750;
    private int higherPWM = 1150;
    private double servoNotReleasedPosition = 1.0;
    private double servoReleasedPosition = 0;
    public Drone(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        droneServo = hwMap.get(ServoImplEx.class, "droneServo");
        droneServo.setPwmRange(new PwmControl.PwmRange(lowerPWM, higherPWM));

    }

    public void setState(){
        selectTransition();
    }

    private void selectTransition(){
        String desiredState = (String) stateMap.get(DRONE_SYSTEM_NAME);

        switch(desiredState){
            case DRONE_NOT_RELEASED:{
                droneNotReleased();
                break;
            }
            case DRONE_RELEASED:{
                droneRelease();
                break;
            }
        }
    }

    private void droneNotReleased(){
        droneServo.setPosition(servoNotReleasedPosition);
    }

    private void droneRelease(){
        droneServo.setPosition(servoReleasedPosition);
    }
}
