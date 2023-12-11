package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Drawbridge {
    public final String DRAWBRIDGE_SYSTEM_NAME = "DRAWBRIDGE_SYSTEM_NAME";
    public final String DRAWBRIDGE_UP_STATE = "DRAWBRIDGE_UP_STATE";
    public final String DRAWBRIDGE_DOWN_STATE = "DRAWBRIDGE_DOWN_STATE";
    public final String DRAWBRIDGE_1_PIXEL_HEIGHT = "DRAWBRIDGE_1_PIXEL_HEIGHT";
    public final String DRAWBRIDGE_2_PIXEL_HEIGHT = "DRAWBRIDGE_2_PIXEL_HEIGHT";
    public final String DRAWBRIDGE_3_PIXEL_HEIGHT = "DRAWBRIDGE_3_PIXEL_HEIGHT";
    public final String DRAWBRIDGE_4_PIXEL_HEIGHT = "DRAWBRIDGE_4_PIXEL_HEIGHT";
    public final String DRAWBRIDGE_5_PIXEL_HEIGHT = "DRAWBRIDGE_5_PIXEL_HEIGHT";
    private int drawbridgePWMLowerLimit = 1230;
    private int drawbridgePWMHigherLimit = 1950;

    private int hardstopPWMLowerLimit = 550;
    private int hardstopPWMHigherLimit = 2500;
    private double firstPixelPosition = 0.99;
    private double secondPixelPosition = 0.95;
    private double thirdPixelPosition = 0.7;
    private double fourthPixelPosition = 0.22;
    private double fifthPixelPosition = 0.01;

    // 550 is 5th pixel pwm
    // 2420 is 1st pixel pwm
    // 2180 for 2nd pixel pwm
    // 1520 for 3rd pixel
    //960 for 4th pixel

    private Telemetry telemetry;

    private Map stateMap;

    private ServoImplEx drawBridge;

    private ServoImplEx hardStop;

    private double drawBridgeUpPosition = 1;

    private double drawBridgeDownPosition = 0;

    public Drawbridge(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        drawBridge = hwMap.get(ServoImplEx.class, "Drawbridge");
        hardStop = hwMap.get(ServoImplEx.class, "intakeHardstop");

        drawBridge.setPwmRange(new PwmControl.PwmRange(drawbridgePWMLowerLimit, drawbridgePWMHigherLimit));
        hardStop.setPwmRange(new PwmControl.PwmRange(hardstopPWMLowerLimit, hardstopPWMHigherLimit));

    }

    public void setState(){
        selectTransition();
    }

    private void selectTransition(){
        String state = (String) stateMap.get(DRAWBRIDGE_SYSTEM_NAME);
        telemetry.addData("State of drawbridge", state);
        switch(state){
            case DRAWBRIDGE_UP_STATE:{
                setDrawBridgeUp();
                setHardstopPosition(fifthPixelPosition);
                break;
            }
            case DRAWBRIDGE_DOWN_STATE:{
                setDrawBridgeDown();
                break;
            }
            case DRAWBRIDGE_1_PIXEL_HEIGHT:{
                setDrawBridgeDown();
                setHardstopPosition(fifthPixelPosition);
                break;
            }
            case DRAWBRIDGE_2_PIXEL_HEIGHT:{
                setDrawBridgeDown();
                setHardstopPosition(secondPixelPosition);
                break;
            }
            case DRAWBRIDGE_3_PIXEL_HEIGHT:{
                setDrawBridgeDown();
                setHardstopPosition(thirdPixelPosition);
                break;
            }
            case DRAWBRIDGE_4_PIXEL_HEIGHT:{
                setDrawBridgeDown();
                setHardstopPosition(fourthPixelPosition);
                break;
            }
            case DRAWBRIDGE_5_PIXEL_HEIGHT:{
                setDrawBridgeDown();
                setHardstopPosition(fifthPixelPosition);
                break;
            }
        }
    }

    public void setDrawBridgeUp(){
        telemetry.addData("Drawbridge State", "up");
        drawBridge.setPosition(1);
    }
    public void setDrawBridgeDown(){
        telemetry.addData("Drawbridge State", "down");
        drawBridge.setPosition(0);
    }

    public void setHardstopPosition(double position){
        hardStop.setPosition(position);
    }
}
