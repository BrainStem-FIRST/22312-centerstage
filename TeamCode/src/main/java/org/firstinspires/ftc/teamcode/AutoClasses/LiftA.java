package org.firstinspires.ftc.teamcode.AutoClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.PIDController;

import java.util.HashMap;
import java.util.Map;
public class LiftA {
    private Telemetry telemetry;
    public DcMotorEx liftMotor1;
    private PIDController liftController;

    // Encoder Positions for Lift heights
    private final int LIFT_GROUND_STATE_POSITION = 0;
    private final int LIFT_LOW_STATE_POSITION = 300;
    private final int LIFT_MIDDLE_STATE_POSITION = 550;
    private final int LIFT_HIGH_STATE_POSITION = 700;

    private final int LIFT_IDLE_STATE_POSITION = 140;


    //Other important variables
    private final int heightTolerance = 5;

    public LiftA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;


        liftController = new PIDController(0,0,0);
        liftMotor1 = hwMap.get(DcMotorEx.class, "liftMotor1");

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        liftController.setInputBounds(LIFT_GROUND_STATE_POSITION, LIFT_HIGH_STATE_POSITION);
//        liftController.setOutputBounds(0,1);
    }

    private void raiseHeightTo(int desiredTickPosition){
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setTargetPosition(desiredTickPosition);
        liftMotor1.setPower(0.5);
    }
    private void moveToPID(int desiredTickPosition){
        int currentPosition = liftMotor1.getCurrentPosition();
        int error = Math.abs(currentPosition - desiredTickPosition);
        if(error < 7){
            liftMotor1.setTargetPosition(desiredTickPosition);
            liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setRawPower(1.0);
        }else{
            setMotorPIDPower(desiredTickPosition, currentPosition);
        }
    }

    private void setMotorPIDPower(int ticks, int currentPosition){
        if(ticks != liftController.getTarget()) {
            liftController.reset();
            liftController.setTarget(ticks);
        }
        double power = (liftController.update(currentPosition));
        if(currentPosition > ticks) {
            setRawPower(-power);
        }else{
            setRawPower(power);
        }
    }

    public void setRawPower(double power) {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor1.setPower(power);
    }
}
