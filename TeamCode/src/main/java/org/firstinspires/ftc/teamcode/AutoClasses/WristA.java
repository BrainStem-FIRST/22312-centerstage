package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristA {

    ServoImplEx wristServo;

    private int minPWM = 760;
    private int maxPWM = 1771;

    private Telemetry telemetry;

    private double zero_degree_position = 0.0;
    private double fourty_five_degree_position = 0.25;
    private double ninety_degree_position = 0.5;
    private double one_thirty_five_degree_position = 0.75;
    private double one_eighty_degree_position = 1.0;

    public WristA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        wristServo = hwMap.get(ServoImplEx.class, "wristServo");
        wristServo.setPwmRange(new PwmControl.PwmRange(minPWM, maxPWM));
    }

    public void wristToPickUpPosition(){
        wristServo.setPosition(ninety_degree_position);
    }

    public void wristToOneEightyDegreePosition(){
        wristServo.setPosition(1.0);
    }

    public void wristToZeroDegreePosition(){
        wristServo.setPosition(0);
    }

    public Action turnWristOneEighty = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    wristToOneEightyDegreePosition();
                    return false;
                }
            },
            new SleepAction(1.5)
    );

    public Action turnWristNinety =  new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    wristToPickUpPosition();
                    return false;
                }
            },
            new SleepAction(1.5)
    );

    public Action turnWristZero = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    wristToZeroDegreePosition();
                    return false;
                }
            },
            new SleepAction(1.5)
    );
}