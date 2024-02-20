package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name="Robot: 22312Tele", group="Robot")
public class RobotTeleOp extends LinearOpMode {
    private int liftCounter = 2;
    private int rightButtonCounter = 0;
    private int leftButtonCounter = 0;
    private int bumperPushes = 0;
    private int drawbridgeCounter = 0;
    private int wristPositionCounter = 2;
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
    private StickyButton gamepad1leftbutton = new StickyButton();
    private StickyButton gamepad1xbutton = new StickyButton();
    private StickyButton dpadUp = new StickyButton();
    private StickyButton dpadDown = new StickyButton();
    private StickyButton dpad_right = new StickyButton();
    private StickyButton dpad_left = new StickyButton();

    private boolean isReset = false;

    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
            put(GAMEPAD_1_A_STATE, false);
            put(GAMEPAD_1_A_IS_PRESSED, false);
        }};
    @Override
    public void runOpMode() {
        Constants constants = new Constants();
        Map<String, String> stateMap = new HashMap<String, String>() {{
        }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
        stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_90_DEGREE_STATE);
        stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
        stateMap.put(robot.transfer.TRANSFER_SYSTEM_NAME, robot.transfer.TRANSFER_GATE_OPEN);
        stateMap.put(robot.hopper.HOPPER_SYSTEM_NAME, robot.hopper.HOPPER_NO_PIXELS);
        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
        stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
//        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
        stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW3_STATE);
//        stateMap.put(robot.drone.DRONE_SYSTEM_NAME, robot.drone.DRONE_NOT_RELEASED);
        robot.updateSystems();
        waitForStart();


        while (!isStopRequested()) {
            setButtons();
            if(gamepad1.right_trigger > 0.5){
                stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
            }
            if (gamepad2.x) {
                stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
            }
            if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_HEIGHT));
                stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
                if (gamepad1.a) {
                    wristPositionCounter = 0;
                }
            } else {
                if (gamepad1.a) {
                    retractionInProgress = true;
                    retractionTime.reset();
                }
                if (!retractionInProgress) {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
                }
            }
            if (retractionInProgress) {
                if (retractionTime.seconds() > 0.5) {
                    wristPositionCounter = 2;
                    stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
                }
                if (retractionTime.seconds() > 1.5) {
                    toggleMap.put(GAMEPAD_1_A_STATE, false);
                    retractionInProgress = false;
                }
            }
            gamepad1leftbutton.update(gamepad1.left_bumper);
            gamepad1rightbutton.update(gamepad1.right_bumper);
            if (gamepad1leftbutton.getState() && bumperPushes != 2) {
                bumperPushes++;
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.GREEN_DEPOSITER_OPEN);
            }
            if (gamepad1leftbutton.getState() && bumperPushes == 2) {
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
                bumperPushes = 0;
                if (gamepad1.left_bumper) {
                    retractionInProgress = true;
                    retractionTime.reset();
                }
            }
            if (gamepad1rightbutton.getState() && bumperPushes != 2) {
                bumperPushes++;
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.RED_DEPOSITER_OPEN);
            }
            if (gamepad1rightbutton.getState() && bumperPushes == 2) {
                stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
                bumperPushes = 0;
                if (gamepad1.left_bumper) {
                    retractionInProgress = true;
                    retractionTime.reset();
                }
            }
            gamepad2RightButton.update(gamepad2.right_bumper);
            gamepad2LeftButton.update(gamepad2.left_bumper);
            if (gamepad2RightButton.getState() && !hangingMode) {
                liftCounter += 1;
            }
            if (gamepad2LeftButton.getState() && !hangingMode) {
                if (liftCounter != 0) {
                    liftCounter -= 1;
                }
            }
            dpad_left.update(gamepad1.dpad_left);
            dpad_right.update(gamepad1.dpad_right);
            if (dpad_left.getState()) {
                wristPositionCounter += 1;
            }
            if (dpad_right.getState()) {
                wristPositionCounter -= 1;
            }
            if (robot.lift.liftMotor2.getCurrentPosition() > 230) {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                (0.6 * -gamepad1.left_stick_y),
                                (0.6 * -gamepad1.left_stick_x)
                        ),
                        (0.2 * -gamepad1.right_stick_x)));
            } else {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x));
            }

            robot.drive.updatePoseEstimate();
            updateWristPosition(stateMap, robot);
            updateLift(stateMap, robot);
            robot.updateSystems();
            telemetry.addData("retraction time", retractionTime.seconds());
            telemetry.addData("Bumper pushes", bumperPushes);
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
