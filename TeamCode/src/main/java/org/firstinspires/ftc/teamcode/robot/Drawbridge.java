package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
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
    private int drawbridgePWMLowerLimit = 500;
    private int drawbridgePWMHigherLimit = 2500;

    private int hardstopPWMLowerLimit = 500;
    private int hardstopPWMHigherLimit = 2000;
    private double firstPixelPosition = 0.01;
    private double secondPixelPosition = 0.5;
    private double thirdPixelPosition = 0.01;
//    private double fourthPixelPosition = 0.22;
//    private double fifthPixelPosition = 0.01;
    private double drawBridge5thPixelPosition = 0.99;
    private double drawBridge4thPixelPosition = 0.117;

    // 120 is 5th pixel pwm
    // 2470 is 1st pixel pwm
    // 1889 for 2nd pixel pwm
    // 1143 for 3rd pixel
    //667 for 4th pixel
    //745 for 4th drawbridge pwm
    //760 for 5th drawbridge pwm

    private Telemetry telemetry;

    private Map stateMap;

    public ServoImplEx drawBridge;

    public ServoImplEx hardStop;

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
                setHardstopPosition(firstPixelPosition);
                break;
            }
            case DRAWBRIDGE_DOWN_STATE:{
                setDrawBridgeDown();
                break;
            }
            case DRAWBRIDGE_1_PIXEL_HEIGHT:{
                setDrawBridgeDown();
                setHardstopPosition(firstPixelPosition);
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
                setDrawbridgePosition(drawBridge4thPixelPosition);
                break;
            }
            case DRAWBRIDGE_5_PIXEL_HEIGHT:{
//                setDrawbridgePosition(drawBridge5thPixelPosition);
                setHardstopPosition(drawBridge5thPixelPosition);
                break;
            }
        }
    }

    public void setDrawBridgeUp(){
        telemetry.addData("Drawbridge State", "up");
        drawBridge.setPosition(0.01);
    }
    public void setDrawBridgeDown(){
        telemetry.addData("Drawbridge State", "down");
        drawBridge.setPosition(0.99);
    }

    public void setHardstopPosition(double position){
        hardStop.setPosition(position);
    }

    public void setDrawbridgePosition(double position){
        drawBridge.setPosition(position);
    }
}
