package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SafePathBuilder;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;
public class IntakeA {
    private Telemetry telemetry;

    public DcMotorEx intakeMotor;

    public ElapsedTime cycleSpitTime = new ElapsedTime();

    public ElapsedTime timeBetweenIntakeSpit = new ElapsedTime();

    public IntakeA(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // This method is for Auto use only
    public void spitPixelinAuto() {
        // Start rotating motor
        intakeMotor.setPower(-1.0); //35);
    }

    public void intakePixelinAuto() {
        intakeMotor.setPower(1.0);
    }

    public  void stopIntakeinAuto() {
        // Stop motor
        intakeMotor.setPower(0);
    }

    public Action spitPixel = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    spitPixelinAuto();
                    return false;
                }
            },
            new SleepAction(0.5),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    stopIntakeinAuto();
                    return false;
                }
            }
    );

    public Action intakePixel = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakePixelinAuto();
                    return false;
                }
            },
            new SleepAction(5.5),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    stopIntakeinAuto();
                    return false;
                }
            }
    );

    public Action intakePixelSecondTime = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(1.0);
                    return false;
                }
            },
            new SleepAction(4.5),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    stopIntakeinAuto();
                    return false;
                }
            }
    );

    public Action intakePixelLeftPixelSpike = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakePixelinAuto();
                    return false;
                }
            },
            new SleepAction(4),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    stopIntakeinAuto();
                    return false;
                }
            }
    );

    public Action intakePixelSecondPassPixelSpikeLeft = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(1.0);
                    return false;
                }
            },
            new SleepAction(4),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    stopIntakeinAuto();
                    return false;
                }
            }
    );
}
