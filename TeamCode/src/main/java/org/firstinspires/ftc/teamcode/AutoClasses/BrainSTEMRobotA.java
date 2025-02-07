package org.firstinspires.ftc.teamcode.AutoClasses;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSensor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class BrainSTEMRobotA {

    private ElapsedTime elapsedTime = new ElapsedTime();
    private Telemetry telemetry;

    public HuskyLens huskyLens;

    public IntakeA intake;
    public DrawbridgeA drawbridge;
    public HopperA hopper;
    public LiftA lift;
    public ArmA arm;
    public WristA wrist;

    public DepositorA depositor;

    public boolean intakePixelSensor = false;
    public NormalizedColorSensor colorSensor;
    public NormalizedColorSensor colorSensor1;
    public NormalizedColorSensor colorSensor2;
    public NormalizedRGBA colors;

    public MecanumDrive drive;

    public BrainSTEMRobotA(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        intake = new IntakeA(hardwareMap, telemetry);
        drawbridge = new DrawbridgeA(hardwareMap, telemetry);
        hopper = new HopperA(hardwareMap, telemetry);
        lift = new LiftA(hardwareMap, telemetry);
        arm = new ArmA(hardwareMap, telemetry);
        depositor = new DepositorA(hardwareMap, telemetry);
        wrist = new WristA(hardwareMap, telemetry);


        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightFloorColorSensor");
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "backPixelColorSensor");
        colorSensor2 = hardwareMap.get(NormalizedColorSensor.class, "frontPixelColorSensor");
        colorSensor.setGain(50);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        telemetry.addData("Robot", "is ready");
        telemetry.update();
    }

    public Action updatePose = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    drive.updatePoseEstimate();
                    return false;
                }
            }
    );
/*
    public Action depositToBoardinAuto = new SequentialAction(
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    grabber.grabPixel();
                    return false;
                }
            },
            new SleepAction(0.5),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    lift.raiseHeightTo(lift.LIFT_GROUND_STATE_POSITION);
                    return false;
                }
            },
            new SleepAction(0.5),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    arm.armToDepositPosition();
                    return false;
                }
            },
            new SleepAction(0.5),
            new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    grabber.depositPixel();
                    return false;
                }
            }
    );

 */

}
