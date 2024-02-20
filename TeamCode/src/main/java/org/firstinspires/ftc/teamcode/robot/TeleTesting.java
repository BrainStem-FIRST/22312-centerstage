package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: TeleTesting", group="Robot")
public class TeleTesting extends LinearOpMode {
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";
    private int wristPositionCounter = 2;

    private StickyButton dpad_right = new StickyButton();
    private StickyButton dpad_left = new StickyButton();

    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
        put(GAMEPAD_1_A_STATE, false);
        put(GAMEPAD_1_A_IS_PRESSED, false);
    }};
    @Override
    public void runOpMode() {

        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        Constants constants = new Constants();
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
        stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_90_DEGREE_STATE);
        stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
        stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
        stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
//        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
        stateMap.put(robot.hopper.HOPPER_SYSTEM_NAME, robot.hopper.HOPPER_NO_PIXELS);
        stateMap.put(robot.transfer.TRANSFER_SYSTEM_NAME, robot.transfer.TRANSFER_GATE_OPEN);
        robot.updateSystems();
        waitForStart();

        while(!isStopRequested()) {
            setButtons();
            if(toggleMap.get(GAMEPAD_1_A_STATE)){
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_ROW5_STATE);
            } else {
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
            }
            dpad_left.update(gamepad1.dpad_left);
            dpad_right.update(gamepad1.dpad_right);
            if(dpad_left.getState()){
                wristPositionCounter += 1;
            }
            if(dpad_right.getState()){
                wristPositionCounter -= 1;
            }
            if(gamepad1.dpad_up){
                stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
            }
            if(gamepad1.dpad_down){
                stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
            }
            if(gamepad1.b){
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
            }
            if(gamepad1.x){
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_PICKUP_ONE);
            }
            if(gamepad1.y){
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_PICKUP_TWO);
            }
            if(gamepad1.right_trigger > 0.5){
                stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
            }

            updateWristPosition(stateMap, robot);
            robot.updateSystems();
            telemetry.addData("Lift motor 2 power", robot.lift.liftMotor2.getPower());
            telemetry.addData("Lift motor 2 encoders",robot.lift.liftMotor2.getCurrentPosition());
            telemetry.addData("Lift motor 1 power", robot.lift.liftMotor1.getPower());
            telemetry.addData("Lift motor 3 power", robot.lift.liftMotor3.getPower());
            telemetry.update();
        }
        }

    private void setButtons() {
        toggleButton(GAMEPAD_1_A_STATE, GAMEPAD_1_A_IS_PRESSED, gamepad1.a);
    }
    private boolean toggleButton(String buttonStateName, String buttonPressName, boolean buttonState) {
        boolean buttonPressed = toggleMap.get(buttonPressName);
        boolean toggle = toggleMap.get(buttonStateName);

        if (buttonState) {
            if (!buttonPressed) {
                toggleMap.put(buttonStateName, !toggle);
                toggleMap.put(buttonPressName, true);
            }
        } else {
            toggleMap.put(buttonPressName, false);
        }

        return toggleMap.get(buttonStateName);
    }

    private void updateWristPosition(Map stateMap, BrainSTEMRobot robot){
        if(wristPositionCounter == 5){
            wristPositionCounter = 0;
        }
        if(wristPositionCounter == -1){
            wristPositionCounter = 4;
        }

        if(wristPositionCounter == 0){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_0_DEGREE_STATE);
        }
        if(wristPositionCounter == 1){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_45_DEGREE_STATE);
        }
        if(wristPositionCounter == 2){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_90_DEGREE_STATE);
        }
        if(wristPositionCounter == 3){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_135_DEGREE_STATE);
        }
        if(wristPositionCounter == 4){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_180_DEGREE_STATE);
        }
    }
}
