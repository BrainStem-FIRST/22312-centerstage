package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.PIDController;

public class LiftA {
    private Telemetry telemetry;
    public DcMotorEx liftMotor2;
    public DcMotorEx liftMotor3;
    public DcMotorEx liftMotor1;
    private PIDController liftController;


    // Encoder Positions for Lift heights
    public final int LIFT_GROUND_STATE_POSITION = 0;
    public final int LIFT_LOW_STATE_POSITION = 124;
    public final int LIFT_MIDDLE_STATE_POSITION = 369;
    public final int LIFT_HIGH_STATE_POSITION = 670;

    public final int LIFT_IDLE_STATE_POSITION = 140;


    //Other important variables
    private final int heightTolerance = 5;

    public LiftA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;


        liftController = new PIDController(0,0,0);
        liftMotor1 = hwMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hwMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor3 = hwMap.get(DcMotorEx.class, "liftMotor3");

        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor3.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftController.setInputBounds(LIFT_GROUND_STATE_POSITION, LIFT_HIGH_STATE_POSITION);
//        liftController.setOutputBounds(0,1);
    }

    public void raiseHeightTo(int desiredTickPosition){
//        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setTargetPosition(desiredTickPosition);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setPower(1.0);
        int currentPosition = liftMotor2.getCurrentPosition();
        telemetry.addData("Current Position=",currentPosition);
        telemetry.addData("target position", desiredTickPosition);
        telemetry.update();
    }

    public boolean inHeightTolerance(int statePosition){
        int currentPosition = liftMotor2.getCurrentPosition();
        telemetry.addData("Current Position=",currentPosition);
        if((statePosition - heightTolerance) <= currentPosition && currentPosition <=  (statePosition + heightTolerance)){
            telemetry.addLine("Lift arrived");
            telemetry.update();
            return true;
        }
        telemetry.addLine("Lift not there yet");
        telemetry.update();
        return false;
    }
    private void moveToPID(int desiredTickPosition){
        int currentPosition = liftMotor2.getCurrentPosition();
        int error = Math.abs(currentPosition - desiredTickPosition);
        if(error < 7){
            liftMotor2.setTargetPosition(desiredTickPosition);
            liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor2.setPower(power);
    }

    public Action raiseLiftAuto = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    setRawPower(0.5);
                    raiseHeightTo(LIFT_LOW_STATE_POSITION);
                    return false;
                }
            }
    );
}
