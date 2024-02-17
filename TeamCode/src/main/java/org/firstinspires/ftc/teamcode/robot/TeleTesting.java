package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: TeleTesting", group="Robot")
public class TeleTesting extends LinearOpMode {
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";
    private int wristPositionCounter = 0;

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
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
        stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_0_DEGREE_STATE);
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
            updateWristPosition(stateMap, robot);
            robot.updateSystems();
            telemetry.addData("Lift Motor 3 power", robot.lift.liftMotor3.getPower());
            telemetry.addData("Lift Motor 2 power", robot.lift.liftMotor2.getPower());
            telemetry.addData("Lift Motor 1 power", robot.lift.liftMotor1.getPower());
            telemetry.addData("Lift Encoders", robot.lift.liftMotor2.getCurrentPosition());
            telemetry.addData("Wrist Position counter", wristPositionCounter);
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
