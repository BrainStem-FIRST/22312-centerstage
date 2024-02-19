package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@Disabled
@TeleOp(name="Robot: 22312Tele", group="Robot")
public class RobotTeleOp extends LinearOpMode {
    private int liftCounter = 0;
    private int rightButtonCounter = 0;
    private int drawbridgeCounter = 0;
    private boolean hangingMode = false;
    private boolean retractionInProgress = false;
    private boolean liftIsGoingUp = false;
    private boolean deposit1Pixel = false;
    private boolean deposit2Pixel = false;
    private ElapsedTime deposit2Pixeltime = new ElapsedTime();
    private ElapsedTime deposit1PixelTime = new ElapsedTime();
    private ElapsedTime fulcrumMovement = new ElapsedTime();
    private ElapsedTime retractionTime = new ElapsedTime();
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";

    private StickyButton gamepad2RightButton = new StickyButton();
    private StickyButton gamepad2LeftButton = new StickyButton();
    private StickyButton gamepad1rightbutton = new StickyButton();
    private StickyButton gamepad1xbutton = new StickyButton();
    private StickyButton dpadUp = new StickyButton();
    private StickyButton dpadDown = new StickyButton();

    private boolean isReset = false;

    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
            put(GAMEPAD_1_A_STATE, false);
            put(GAMEPAD_1_A_IS_PRESSED, false);
        }};
    @Override
    public void runOpMode() {
        ElapsedTime initTimer = new ElapsedTime();
        Constants constants = new Constants();
        Map<String, String> stateMap = new HashMap<String, String>() {{
        }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
        stateMap.put(robot.hopper.HOPPER_SYSTEM_NAME, robot.hopper.HOPPER_NO_PIXELS);
        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
        stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
        stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW3_STATE);
        stateMap.put(robot.drone.DRONE_SYSTEM_NAME, robot.drone.DRONE_NOT_RELEASED);
        robot.updateSystems();
        waitForStart();

//        robot.lift.raiseHeightTo(800);
//        sleep(500);
//        robot.arm.armToIdlePosition();
//        sleep(200);
//        robot.lift.raiseHeightTo(0);
//        isReset = true;

        while (!isStopRequested()) {
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
    private void updateLift(Map stateMap, BrainSTEMRobot robot){
        Constants constants = new Constants();
        if(liftCounter == 5){
            liftCounter = 0;
        }
        if(liftCounter == 0){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW1_STATE);
        }
        if(liftCounter == 1){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW2_STATE);
        }
        if(liftCounter == 2){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW3_STATE);
        }
        if(liftCounter == 3){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW4_STATE);
        }
        if(liftCounter == 4){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW5_STATE);
        }
    }

    private void updateDrawbridge(Map stateMap, BrainSTEMRobot robot){
        if(drawbridgeCounter == 6){
            drawbridgeCounter = 1;
        }
        if(drawbridgeCounter == 1){
            stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_1_PIXEL_HEIGHT);
        }
        if(drawbridgeCounter == 2){
            stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_2_PIXEL_HEIGHT);
        }
        if(drawbridgeCounter == 3){
            stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_3_PIXEL_HEIGHT);
        }
        if(drawbridgeCounter == 4){
            stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_4_PIXEL_HEIGHT);
        }
        if(drawbridgeCounter == 5){
            stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_5_PIXEL_HEIGHT);
        }
    }
}
