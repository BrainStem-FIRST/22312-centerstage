package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;
public class Lift {
    private Telemetry telemetry;
    public DcMotorEx liftMotor2;
    public DcMotorEx liftMotor1;
    public DcMotorEx liftMotor3;
    private Map stateMap;
    //Create Strings for StateMap Lift Control
    public final String LIFT_SYSTEM_NAME = "LIFT_SYSTEM_NAME";
    public final String LIFT_GROUND_STATE = "LIFT_GROUND_STATE";
    public final String LIFT_ROW1_STATE = "LIFT_ROW1_STATE";
    public final String LIFT_ROW2_STATE = "LIFT_ROW2_STATE";
    public final String LIFT_ROW3_STATE = "LIFT_ROW3_STATE";
    public final String LIFT_ROW4_STATE = "LIFT_ROW4_STATE";
    public final String LIFT_ROW5_STATE = "LIFT_ROW5_STATE";
    public final String LIFT_ROW6_STATE = "LIFT_ROW6_STATE";
    public final String LIFT_ROW7_STATE = "LIFT_ROW7_STATE";
    public final String LIFT_ROW8_STATE = "LIFT_ROW8_STATE";
    public final String LIFT_ROW9_STATE = "LIFT_ROW9_STATE";
    public final String LIFT_ROW10_STATE = "LIFT_ROW10_STATE";
    public final String LIFT_ROW11_STATE = "LIFT_ROW11_STATE";
    public final String LIFT_ROW12_STATE = "LIFT_ROW12_STATE";
    public final String LIFT_ROW13_STATE = "LIFT_ROW13_STATE";
    public final String LIFT_ROW14_STATE = "LIFT_ROW14_STATE";
    public final String LIFT_ROW15_STATE = "LIFT_ROW15_STATE";
    public final String LIFT_ROW16_STATE = "LIFT_ROW16_STATE";
    public final String LIFT_ROW17_STATE = "LIFT_ROW17_STATE";




    public final String LIFT_IDLE_STATE = "LIFT_IDLE_STATE";

    public final String TRANSITION_STATE = "TRANSITION_STATE";

    public final String LIFT_CURRENT_STATE = "LIFT_CURRENT_STATE";

    public final String LIFT_DESIRED_POSITION = "LIFT_DESIRED_POSITION";

    private double armMinimumAngle = 0.08;

    // Encoder Positions for Lift heights
    //Lift row 1: 273
    //Lift row 2: 514
    //lift row 3: 755
    //lift row 4: 996
    //lift row 5: 1044
    private int LIFT_GROUND_STATE_POSITION = 0;
    public int LIFT_IDLE_STATE_POSITION = 100;
    private int LIFT_ROW1_POSITION = 388;
    private int LIFT_ROW2_POSITION = 584;
    private int LIFT_ROW3_POSITION = 737;
    private int LIFT_ROW4_POSITION = 959;
    private int LIFT_ROW5_POSITION = 1120;
    private int LIFT_ROW_6_POSITION = 1415;
    private int LIFT_ROW_7_POSITION = 1525;
    private int LIFT_ROW_8_POSITION = 1740;
//    private int LIFT_ROW_9_POSITION = 1170;
//    private int LIFT_ROW_10_POSITION = 1320;
//    private int LIFT_ROW_11_POSITION = 1470;
//    private int LIFT_ROW_12_POSITION = 1620;
//    private int LIFT_ROW_13_POSITION = 1740;
    private int LIFT_ROW_14_POSITION = 1920;
    private int LIFT_ROW_15_POSITION = 2070;
    private int LIFT_ROW_16_POSITION = 2220;
    private int LIFT_ROW_17_POSITION = 2370;


    //PID values
    private double kP = 0.0065;
    private double kI = 0;
    private double kD = 0;
    private double kS = 0.02;

    private double lowerkP = 0.0095;

    PIDController liftController = new PIDController(kP,kI,kD);
    PIDController liftControllerDown = new PIDController(lowerkP, kI, kD);
    //Other important variables
    private int heightTolerance = 3;

    private int cycleTolerance = 5;

    public ElapsedTime liftCycleTime = new ElapsedTime();
    private Constants constants = new Constants();
    public Lift(HardwareMap hwMap, Telemetry telemetry, Map stateMap){
        this.telemetry = telemetry;
        this.stateMap = stateMap;

        liftMotor2 = hwMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor1 = hwMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor3 = hwMap.get(DcMotorEx.class, "liftMotor3");

//        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        liftMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor3.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftController.setInputBounds(LIFT_GROUND_STATE_POSITION, LIFT_ROW5_POSITION);
        liftController.setOutputBounds(-0.9,0.8);

        liftControllerDown.setInputBounds(LIFT_GROUND_STATE_POSITION, LIFT_ROW5_POSITION);
        liftControllerDown.setOutputBounds(-0.9,0.8);

    }

    public void setState(Arm arm){
        String hangingMode  = (String) stateMap.get(constants.HANGING_MODE);
        if(hangingMode.equalsIgnoreCase("false")){
            updatePixelCycleState();

            String currentLevel = getCurrentState();
            String desiredLevel = (String) stateMap.get(LIFT_SYSTEM_NAME);

            int desiredPosition = selectTransition(desiredLevel, arm);
            boolean liftGoingUp = desiredPosition > liftMotor2.getCurrentPosition();

            telemetry.addData("Lift going up", liftGoingUp);

            stateMap.put(LIFT_CURRENT_STATE, currentLevel);
            stateMap.put(LIFT_DESIRED_POSITION, selectTransition(desiredLevel, arm));
            if(desiredLevel.equalsIgnoreCase(currentLevel)) {
                liftController.updateWithError(0);
                liftControllerDown.updateWithError(0);
            } else {
                if(liftGoingUp){
                    raiseHeightPID(desiredPosition);
                } else{
                    lowerHeightPID(desiredPosition);
                }
            }
        }
    }

    private void updatePixelCycleState(){
        if(isCycleInProgress()){
            if(inCycleTolerance(liftMotor2.getCurrentPosition(), LIFT_GROUND_STATE_POSITION) || liftCycleTime.milliseconds() > 50){
                stateMap.put(constants.PIXEL_CYCLE_LIFT_DOWN, constants.PIXEL_CYCLE_STATE_COMPLETE);
                stateMap.put(constants.PICKUP_DELAY_TIMESTART, System.currentTimeMillis());
            }
        }
    }

    public void resetEncoders(){
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private boolean inCycleTolerance(int currentPosition, int statePosition){
        if((statePosition - cycleTolerance) <= currentPosition && currentPosition <=  (statePosition + cycleTolerance)){
            return true;
        }
        return false;
    }
    private boolean isCycleInProgress(){
        String liftDownCycleState = (String) stateMap.get(constants.PIXEL_CYCLE_LIFT_DOWN);
        if(liftDownCycleState.equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
            return true;
        }
        return false;
    }
    private int getDesiredPosition(String desiredLevel){
        if(desiredLevel.equals(LIFT_GROUND_STATE)){
            return LIFT_ROW5_POSITION;
        } else if(desiredLevel.equals(LIFT_IDLE_STATE)){
            return LIFT_IDLE_STATE_POSITION;
        } else if(desiredLevel.equals(LIFT_ROW1_STATE)){
            return LIFT_ROW1_POSITION;
        } else if(desiredLevel.equals(LIFT_ROW2_STATE)){
            return LIFT_ROW2_POSITION;
        } else if(desiredLevel.equals(LIFT_ROW3_STATE)){
            return LIFT_ROW3_POSITION;
        } else if(desiredLevel.equals(LIFT_ROW4_STATE)){
            return LIFT_ROW4_POSITION;
        } else {
            return LIFT_ROW5_POSITION;
        }
    }
    private String getCurrentState() {
        String state = TRANSITION_STATE;
        int currentPosition = liftMotor2.getCurrentPosition();
        if(inHeightTolerance(currentPosition, LIFT_GROUND_STATE_POSITION)){
            telemetry.addData("In height tolerance", "ground");
            state = LIFT_GROUND_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW1_POSITION)){
            telemetry.addData("In height tolerance", "low");
            state = LIFT_ROW1_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW2_POSITION)){
            telemetry.addData("In height tolerance", "middle");
            state = LIFT_ROW2_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW3_POSITION)){
            telemetry.addData("In height tolerance", "high");
            state = LIFT_ROW3_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_IDLE_STATE_POSITION)){
            telemetry.addData("In height tolerance", "idle");
            state = LIFT_IDLE_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW4_POSITION)){
            state = LIFT_ROW4_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW5_POSITION)){
            state = LIFT_ROW5_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW_6_POSITION)){
            state = LIFT_ROW6_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW_7_POSITION)){
            state = LIFT_ROW7_STATE;
        } else if(inHeightTolerance(currentPosition, LIFT_ROW_8_POSITION)){
            state = LIFT_ROW8_STATE;
        }
        return state;
    }

    private boolean inHeightTolerance(int currentPosition, int statePosition){
        if((statePosition - heightTolerance) <= currentPosition && currentPosition <=  (statePosition + heightTolerance)){
            return true;
        }
        return false;
    }

    private int selectTransition(String desiredState, Arm arm){
        int target = 0;
        switch (desiredState){
            case LIFT_GROUND_STATE:{
                target =  LIFT_GROUND_STATE_POSITION;
                break;
            }
            case LIFT_ROW1_STATE:{
                target =  LIFT_ROW1_POSITION;
                break;
            }
            case LIFT_ROW2_STATE:{
                target =  LIFT_ROW2_POSITION;
                break;
            }
            case LIFT_ROW3_STATE:{
                target =  LIFT_ROW3_POSITION;
                break;
            }
            case LIFT_ROW4_STATE:{
                target = LIFT_ROW4_POSITION;
                break;
            }
            case LIFT_ROW5_STATE:{
                target = LIFT_ROW5_POSITION;
                break;
            }
            case LIFT_ROW6_STATE:{
                target = LIFT_ROW_6_POSITION;
                break;
            }
            case LIFT_ROW7_STATE:{
                target = LIFT_ROW_7_POSITION;
                break;
            }
            case LIFT_ROW8_STATE:{
                target = LIFT_ROW_8_POSITION;
                break;
            }
            case LIFT_IDLE_STATE:{
                target = LIFT_IDLE_STATE_POSITION;
                break;
            }

        }
        return target;
    }

    public void raiseHeightTo(int desiredTickPosition){
        liftMotor2.setTargetPosition(desiredTickPosition);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setPower(0.5);
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

    public void raiseHeightPID(int desiredTickPosition){
        liftController.setTarget(desiredTickPosition);
        double error = desiredTickPosition - liftMotor2.getCurrentPosition();
        telemetry.addData("Error for lift", error);
        setRawPower(liftController.updateWithError(error) + kS);
        telemetry.addData("Lift motor power", liftMotor2.getPower());
    }

    public void lowerHeightPID(int desiredTickPosition){
        liftControllerDown.setTarget(desiredTickPosition);
        double error = desiredTickPosition - liftMotor2.getCurrentPosition();
        telemetry.addData("Error for lift", error);
        setRawPower(liftControllerDown.updateWithError(error) + kS);
        telemetry.addData("Lift motor power", liftMotor2.getPower());
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
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setPower(power);
        liftMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor3.setPower(power);
    }

    public void setZeroBrakeBehavior(){
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
