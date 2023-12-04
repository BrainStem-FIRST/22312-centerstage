package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Arm {
    public final String ARM_SYSTEM_NAME = "ARM_SYSTEM_NAME";
    public final String ARM_DEPOSIT_STATE = "ARM_DEPOSIT_STATE";
    public final String ARM_IDLE_STATE = "ARM_IDLE_STATE";
    public ServoImplEx rightArmServo;
    public ServoImplEx leftArmServo;
    private double arm_deposit_position = 1.0;
    private double arm_idle_position = 0.0;
    private Telemetry telemetry;
    private Map stateMap;

    public AnalogInput armEncoder;
    private Lift lift;

    public AbsoluteAnalogEncoder encoder;
    private double leftServoLowerPWMLimit = 930;
    private double  leftServoHigherPWMLimit= 2520;

    private double rightServoPWMHigherLimit = 2093;
    private double rightServoPWMLowerLimit = 531;

    private double rightDepositPosition = 0;
    private double leftDepositPosition = 1.0;
    private double rightIdlePosition = 1.0;
    private double leftIdlePosition = 0.0;
    private double encoderOffset = 1.53;

    private int liftMinPosition = 200;

    public Arm(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        rightArmServo = hwMap.get(ServoImplEx.class, "rightArmServo");
        leftArmServo = hwMap.get(ServoImplEx.class, "leftArmServo");

        rightArmServo.setPwmRange(new PwmControl.PwmRange(rightServoPWMLowerLimit,rightServoPWMHigherLimit));
        leftArmServo.setPwmRange(new PwmControl.PwmRange(leftServoLowerPWMLimit,leftServoHigherPWMLimit));

        armEncoder = hwMap.get(AnalogInput.class, "armEncoder");

        encoder = new AbsoluteAnalogEncoder(armEncoder, 3.3).zero(encoderOffset).setInverted(false);

    }

    public void setState(Lift lift){
        if(lift.liftMotor1.getCurrentPosition() > liftMinPosition){
            selectTransition();
        }
    }

    private void selectTransition(){
        String desiredState = (String) stateMap.get(ARM_SYSTEM_NAME);
        switch (desiredState){
            case ARM_DEPOSIT_STATE:{
                armToDepositPosition();
                break;
            }

            case ARM_IDLE_STATE:{
                armToIdlePosition();
                break;
            }
        }

    }

    private void armToDepositPosition(){
//        leftArmServo.setPosition(leftIdlePosition);
        telemetry.addData("Arm Position Called", "Deposit");
        rightArmServo.setPosition(0.03);
        leftArmServo.setPosition(0.9);
    }
    private void armToIdlePosition(){
//        leftArmServo.setPosition(leftDepositPosition);
        telemetry.addData("Arm Position Called", "Idle");
        rightArmServo.setPosition(0.9);
        leftArmServo.setPosition(0.03);

    }
}
