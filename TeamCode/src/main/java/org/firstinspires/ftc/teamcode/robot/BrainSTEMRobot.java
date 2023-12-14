package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Pose2d;
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
    public Fulcrum fulcrum;
    public Intake intake;
    public Drawbridge drawbridge;

    public MecanumDrive drive;
    public Arm arm;
    public Grabber grabber;
    public Hanging hanging;
    public GrabberCR grabberCR;

    public Drone drone;
    private Map stateMap;
    Constants constants = new Constants();
    public BrainSTEMRobot(HardwareMap hardwareMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap =  stateMap;

        hopper = new Hopper(hardwareMap, telemetry, stateMap);
        intake = new Intake(hardwareMap, telemetry, stateMap, hopper);
        fulcrum = new Fulcrum(hardwareMap, telemetry, stateMap);
        drawbridge = new Drawbridge(hardwareMap, telemetry, stateMap);
        arm = new Arm(hardwareMap, telemetry, stateMap);
        lift = new Lift(hardwareMap, telemetry, stateMap);
        hanging = new Hanging(hardwareMap, telemetry, stateMap);
//        grabberCR = new GrabberCR(hardwareMap, telemetry, stateMap);
        grabber = new Grabber(hardwareMap, telemetry, stateMap);
        drone = new Drone(hardwareMap, telemetry, stateMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_INTAKE_SPITTING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_LIFT_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        stateMap.put(constants.PIXEL_CYCLE_GRABBER, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        telemetry.addData("Robot", "is ready");
        telemetry.update();
    }

    public void updateSystems(){
        telemetry.addData("Robot stateMap", stateMap);
        String pixelPickup = (String)(stateMap.get(constants.PIXEL_CYCLE));

        if(pixelPickup.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            pixelPickupFunction();
        } else {
            hopper.setState();
            intake.setState();
            fulcrum.setState(lift);
            drawbridge.setState();
            arm.setState(lift);
            hanging.setState();
            lift.setState(arm);
            grabber.setState();
            drone.setState();
        }
    }

    private void pixelPickupFunction() {
        telemetry.addData("Start intake function", startIntake());
        if(startFulcrumDown()){
            fulcrum.fulcrumCycleDownTime.reset();
            stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if (startIntake()) {
            stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if(startIntakeSpit()){
            intake.cycleSpitTime.reset();
            stateMap.put(constants.PIXEL_CYCLE_INTAKE_SPITTING, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if(startFulcrumUp()){
            fulcrum.fulcrumCycleUpTime.reset();
            stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if(startLiftDown()){
            lift.LIFT_IDLE_STATE_POSITION = 0;
            lift.liftCycleTime.startTime();
            stateMap.put(constants.PIXEL_CYCLE_LIFT_DOWN, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if(startGrabber()){
            grabber.grabberCycleTime.reset();
            stateMap.put(constants.PIXEL_CYCLE_GRABBER, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
        } else if(isConeCycleComplete()){
            telemetry.addData("Pixel cycle", "Complete");
            stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_GRABBER, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_INTAKE_SPITTING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE_LIFT_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
            stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
        }
        setPixelPickupSubsystems();
    }

    private boolean isConeCycleComplete(){
        String intakeIntakeCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_INTAKING);
        String intakeSpittingCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_SPITTING);
        String pixelCycleFulcrumDown = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN);
        String pixelCycleLiftDown = (String) stateMap.get(constants.PIXEL_CYCLE_LIFT_DOWN);
        String pixelCycleGrabber = (String) stateMap.get(constants.PIXEL_CYCLE_GRABBER);
        String pixelCycleFulcrumUp = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP);
        if(intakeSpittingCycleState.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && intakeIntakeCycleState.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleFulcrumDown.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleLiftDown.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleGrabber.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleFulcrumUp.equals(constants.PIXEL_CYCLE_STATE_COMPLETE)){
            return true;
        }
        return false;
    }

    private boolean startFulcrumDown(){
        String pixelCycleState = (String)(stateMap.get(constants.PIXEL_CYCLE));
        String fulcrumMoveDownState = (String) (stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN));

        if(pixelCycleState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS) && fulcrumMoveDownState.equals(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }
    public boolean startLiftDown(){
        String fulcrumCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP);
        String liftDownCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_LIFT_DOWN);

        if(fulcrumCycleState.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && liftDownCycleState.equals(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }

    private boolean startGrabber(){
        String liftDownPixelState = (String) stateMap.get(constants.PIXEL_CYCLE_LIFT_DOWN);
        String grabberPixelState = (String) stateMap.get(constants.PIXEL_CYCLE_GRABBER);
        if(liftDownPixelState.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && grabberPixelState.equals(constants.PIXEL_CYCLE_STATE_NOT_STARTED) && lift.grabber.grabberCycleDelay.milliseconds() > 600){
            return true;
        }
        return false;
    }
    private boolean startIntakeSpit(){
        String pixelCycleIntakeState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_INTAKING);
        String pixelCycleSpittingState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_SPITTING);
        if(pixelCycleIntakeState.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleSpittingState.equals(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }

    private boolean startFulcrumUp(){
        String pixelCycleIntakeSpittingState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_SPITTING);;
        String pixelCycleFulcrum = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP);
        if(pixelCycleIntakeSpittingState.equals(constants.PIXEL_CYCLE_STATE_COMPLETE) && pixelCycleFulcrum.equals(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }

    private void setPixelPickupSubsystems(){
        intake.setState();
        hopper.setState();
        fulcrum.setState(lift);
        lift.setState(arm);
        grabber.setState();
        drawbridge.setState();;
    }
    private boolean startIntake(){
        String fulcrumMoveDown = (String) stateMap.get(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN);
        String intakeState = (String) stateMap.get(constants.PIXEL_CYCLE_INTAKE_INTAKING);
        if(fulcrumMoveDown.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_COMPLETE) && intakeState.equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
            return true;
        }
        return false;
    }
}
