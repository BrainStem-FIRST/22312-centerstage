package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Hanging {

    public final String HANGING_SYSTEM_NAME = "HANGING_SYSTEM_NAME";

    public final String HANGING_NOT_RELEASED = "HANGING_NOT_RELEASED";

    public final String HANGING_RELEASED = "HANGING_RELEASED";

    private Telemetry telemetry;

    private Map stateMap;

    public DcMotorEx leftHanging;
    public DcMotorEx rightHanging;

    private int lefthangingEncoder = 11141;
    private int rightHangingEncoderTicks = 10931;

    public Encoder rightHangingEncoder;
    public Hanging(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        leftHanging = hwMap.get(DcMotorEx.class, "leftHanging");
        rightHanging = hwMap.get(DcMotorEx.class, "rightHanging");

        leftHanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHanging.setDirection(DcMotor.Direction.REVERSE);
        leftHanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftHanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightHanging.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHanging.setDirection(DcMotor.Direction.REVERSE);
        rightHanging.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setState(){
        String servoState = (String) stateMap.get(HANGING_SYSTEM_NAME);

        switch(servoState){
            case HANGING_NOT_RELEASED:{
                break;
            }
            case HANGING_RELEASED:{
                break;
            }
        }
    }

    private void toDeployPosition(){
        leftHanging.setTargetPosition(lefthangingEncoder);
        leftHanging.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftHanging.setPower(1.0);

//        rightHanging.setTargetPosition(rightHangingEncoderTicks);
//        rightHanging.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightHanging.setPower(1.0);
    }

    private void notReleased(){
        leftHanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightHanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHanging.setPower(0);
//        rightHanging.setPower(0);
    }

    public void setRawPower(double power){
        leftHanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHanging.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHanging.setPower(power);
        rightHanging.setPower(power);
    }
}
