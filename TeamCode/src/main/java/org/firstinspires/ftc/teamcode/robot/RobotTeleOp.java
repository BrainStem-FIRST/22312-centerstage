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

    private boolean liftGoingUp = false;
    private ElapsedTime liftGoingUpTime = new ElapsedTime();

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
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_IDLE_STATE);
        stateMap.put(robot.constants.CURRENT_RIGHT_DEPSOSITER, robot.depositer.RED_DEPOSITER_OPEN);
        stateMap.put(robot.constants.CURRENT_LEFT_DEPSOSITER, robot.depositer.GREEN_DEPOSITER_OPEN);
        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
        stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW3_STATE);
        stateMap.put(constants.HANGING_MODE, "false");
        stateMap.put(robot.drone.DRONE_SYSTEM_NAME, robot.drone.DRONE_NOT_RELEASED);
//        robot.updateSystems();
        waitForStart();


        while (!isStopRequested()) {
            if(gamepad2.right_bumper && gamepad2.left_bumper && !hangingMode){
                robot.lift.resetEncoders();
            } else if(gamepad2.left_trigger > 0.2 && !hangingMode){
                robot.lift.setRawPower(-1.0);
            } else {
                setButtons();
                if(gamepad1.right_trigger > 0.5) {
                    stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
                }
                if(gamepad1.left_trigger > 0.5){
                    stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_LIFT_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_INTAKE_EXTRA, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_DEPOSITER_ONE_WAY_GATE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_DRIVER_INPUT);
                } else if(!((String)(constants.PIXEL_CYCLE)).equalsIgnoreCase(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
                    stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
                }
                if(gamepad2.a && !hangingMode){
                    stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_1_PIXEL);
                }
                if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                    robot.lift.LIFT_IDLE_STATE_POSITION = 100;
                    stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_HEIGHT));
                    stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
                } else {
                    if (gamepad1.a) {
                        retractionInProgress = true;
                        retractionTime.reset();
                    }
                    if (!retractionInProgress) {
                        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_IDLE_STATE);
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
                if(gamepad1leftbutton.getState()){
                    stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, stateMap.get(constants.CURRENT_LEFT_DEPSOSITER));
                    bumperPushes += 1;
                }
                if(gamepad2.right_bumper && gamepad2.b){
                    hangingMode = true;
                    stateMap.put(constants.HANGING_MODE, "true");
                    stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
                }
                if(hangingMode && gamepad2.left_stick_y > 0.5){
                    robot.lift.setRawPower(0.5 * gamepad2.left_stick_y);
                } else if(hangingMode && gamepad2.right_stick_y > 0.5){
                    robot.lift.setRawPower(0.8 * -gamepad2.right_stick_y);
                } else if(hangingMode){
                    robot.lift.setRawPower(0);
                }
                if (gamepad1rightbutton.getState()) {
                    stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
                    if(gamepad1.right_bumper){
                        retractionInProgress = true;
                        retractionTime.reset();
                    }
                }

                if(bumperPushes == 2){
                    stateMap.put(robot.depositer.DEPOSITER_SYSTEM_NAME, robot.depositer.DEPOSITER_OPEN);
                    if(gamepad1.right_bumper || gamepad1.left_bumper){
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
                if(gamepad2.b && hangingMode){
                    stateMap.put(robot.drone.DRONE_SYSTEM_NAME, robot.drone.DRONE_RELEASED);
                }
                dpad_left.update(gamepad1.dpad_left);
                dpad_right.update(gamepad1.dpad_right);
                if (dpad_left.getState()) {
                    wristPositionCounter += 1;
                }
                if (dpad_right.getState()) {
                    wristPositionCounter -= 1;
                }
                gamepad1xbutton.update(gamepad1.x);
                if(gamepad1xbutton.getState() && ((String)stateMap.get(constants.PIXEL_CYCLE)).equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
                    stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_COMPLETE);
                }
                if (robot.lift.liftMotor2.getCurrentPosition() > 230) {
                    robot.drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    (0.6 * -gamepad1.left_stick_y),
                                    (0.6 * -gamepad1.left_stick_x)
                            ),
                            (0.45 * -gamepad1.right_stick_x)));
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
                telemetry.addData("LIft encoder heights", robot.lift.liftMotor2.getCurrentPosition());
                telemetry.addData("Wrist Position counter", wristPositionCounter);
                telemetry.addData("intake motor", robot.intake.intakeMotor.getPower());
                telemetry.addData("hanging mode ", hangingMode);
                telemetry.update();
            }
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
        if(liftCounter == 12){
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
        if(liftCounter == 5){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW6_STATE);
        }
        if(liftCounter == 6){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW7_STATE);
        }
        if(liftCounter == 7){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW8_STATE);
        }
        if(liftCounter == 8){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW9_STATE);
        }
        if(liftCounter == 9){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW10_STATE);
        }
        if(liftCounter == 10){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW11_STATE);
        }
        if(liftCounter == 11){
            stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW12_STATE);
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
            stateMap.put(robot.constants.CURRENT_RIGHT_DEPSOSITER, robot.depositer.RED_DEPOSITER_OPEN);
            stateMap.put(robot.constants.CURRENT_LEFT_DEPSOSITER, robot.depositer.GREEN_DEPOSITER_OPEN);
        }
        if(wristPositionCounter == 1){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_45_DEGREE_STATE);
            stateMap.put(robot.constants.CURRENT_RIGHT_DEPSOSITER, robot.depositer.RED_DEPOSITER_OPEN);
            stateMap.put(robot.constants.CURRENT_LEFT_DEPSOSITER, robot.depositer.GREEN_DEPOSITER_OPEN);
        }
        if(wristPositionCounter == 2){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_90_DEGREE_STATE);
            stateMap.put(robot.constants.CURRENT_RIGHT_DEPSOSITER, robot.depositer.RED_DEPOSITER_OPEN);
            stateMap.put(robot.constants.CURRENT_LEFT_DEPSOSITER, robot.depositer.GREEN_DEPOSITER_OPEN);
        }
        if(wristPositionCounter == 3){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_135_DEGREE_STATE);
            stateMap.put(robot.constants.CURRENT_RIGHT_DEPSOSITER, robot.depositer.GREEN_DEPOSITER_OPEN);
            stateMap.put(robot.constants.CURRENT_LEFT_DEPSOSITER, robot.depositer.RED_DEPOSITER_OPEN);
        }
        if(wristPositionCounter == 4){
            stateMap.put(robot.wrist.WRIST_SYSTEM_NAME, robot.wrist.WRIST_180_DEGREE_STATE);
            stateMap.put(robot.constants.CURRENT_RIGHT_DEPSOSITER, robot.depositer.GREEN_DEPOSITER_OPEN);
            stateMap.put(robot.constants.CURRENT_LEFT_DEPSOSITER, robot.depositer.RED_DEPOSITER_OPEN);

        }
    }
}
