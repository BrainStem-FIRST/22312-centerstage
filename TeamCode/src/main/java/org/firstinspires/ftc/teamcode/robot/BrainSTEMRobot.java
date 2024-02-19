package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Map;

public class BrainSTEMRobot {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private Telemetry telemetry;
    private OpMode opMode;
    public Lift lift;
    public Hopper hopper;
    public Intake intake;
    public Drawbridge drawbridge;

    public MecanumDrive drive;
    public Arm arm;
    public Wrist wrist;
    public Depositer depositer;
    public Transfer transfer;
    public Drone drone;
    private Map stateMap;
    Constants constants = new Constants();
    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap =  stateMap;

        hopper = new Hopper(hardwareMap, telemetry, stateMap);
        intake = new Intake(hardwareMap, telemetry, stateMap, hopper);
//        drawbridge = new Drawbridge(hardwareMap, telemetry, stateMap);
        arm = new Arm(hardwareMap, telemetry, stateMap);
        lift = new Lift(hardwareMap, telemetry, stateMap);
        wrist = new Wrist(hardwareMap, telemetry, stateMap);
//        drone = new Drone(hardwareMap, telemetry, stateMap);
        depositer = new Depositer(hardwareMap, telemetry, stateMap);
        transfer  = new Transfer(hardwareMap, telemetry, stateMap);
//        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PICKUP_DELAY_TIMESTART, System.currentTimeMillis());

        telemetry.addData("Robot", "is ready");
        telemetry.update();
    }

    public void updateSystems(){
        telemetry.addData("Robot stateMap", stateMap);
        String pixelPickup = (String)(stateMap.get(constants.PIXEL_CYCLE));
        if(pixelPickup.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            pixelPickupFunctionSecondVersion();
        } else {
            hopper.setState();
            intake.setState();
//            drawbridge.setState();
            arm.setState(lift);
            lift.setState(arm);
            wrist.setState();
//            drone.setState();
            depositer.setState();
        }
    }

    private void setPixelPickupSubsystems(){
        intake.setState();
        hopper.setState();
        depositer.setState();
        transfer.setState();
//        drawbridge.setState();
    }
    private boolean startIntake(){
        String pixelCycle = (String) stateMap.get(constants.PIXEL_CYCLE);
        String intakeState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_INTAKING);
        if(pixelCycle.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_IN_PROGRESS) && intakeState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }
    private void pixelPickupFunctionSecondVersion(){
        if(startIntake()){
            stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if(startGateAndDepositer()){
            stateMap.put(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
            depositer.depositerCycleTime.reset();
            transfer.transferCycleTime.reset();
        } else if(pixelCycleFunctionComplete()){
            stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        }
        setPixelPickupSubsystems();
    }

    private boolean pixelCycleFunctionComplete(){
        String intakeCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_INTAKING);
        String pixelCycleTransferDepositer = (String) stateMap.get(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE);
        if(intakeCycleState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleTransferDepositer.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_COMPLETE)){
            return true;
        }
        return false;
    }

    private boolean startGateAndDepositer() {
        String intakeCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_INTAKING);
        String depositerCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE);
        long endTime = (long) (stateMap.get(constants.PICKUP_DELAY_TIMESTART)) + 700;
        if (intakeCycleState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_COMPLETE) && depositerCycleState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_NOT_STARTED) && System.currentTimeMillis() > endTime) {
            return true;
        }
        return false;
    }
}
