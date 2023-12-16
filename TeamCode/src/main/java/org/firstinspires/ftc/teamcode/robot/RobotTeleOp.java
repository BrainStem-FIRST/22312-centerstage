package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

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
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
        stateMap.put(robot.hopper.HOPPER_SYSTEM_NAME, robot.hopper.HOPPER_NO_PIXELS);
        stateMap.put(robot.fulcrum.FULCRUM_SYSTEM_NAME, robot.fulcrum.FULCRUM_UP);
        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
        stateMap.put(robot.arm.ARM_SYSTEM_NAME,robot.arm.ARM_IDLE_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME,robot.drawbridge.DRAWBRIDGE_UP_STATE);
        stateMap.put(robot.grabber.GRABBER_SYSTEM_NAME, robot.grabber.GRABBER_MIDDLE_STATE);
        stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW3_STATE);
        stateMap.put(robot.drone.DRONE_SYSTEM_NAME, robot.drone.DRONE_NOT_RELEASED);
        stateMap.put(robot.hanging.HANGING_SYSTEM_NAME, robot.hanging.HANGING_NOT_RELEASED);
        robot.updateSystems();
        waitForStart();

//        robot.lift.raiseHeightTo(800);
//        sleep(500);
//        robot.arm.armToIdlePosition();
//        sleep(200);
//        robot.lift.raiseHeightTo(0);
//        isReset = true;

        while(!isStopRequested()) {
            if (gamepad2.right_bumper && gamepad2.left_bumper && !hangingMode) {
                robot.lift.resetEncoders();
            } else if (gamepad2.left_trigger > 0.5 && !hangingMode) {
                robot.lift.liftMotor1.setPower(-1.0);
            } else {
                setButtons();
                telemetry.addLine("IN Tele");
                if(gamepad2.x){
                    stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME, robot.drawbridge.DRAWBRIDGE_UP_STATE);
                }

                if (gamepad1.left_trigger > 0.5) {
                    stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_UP, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_FULCRUM_MOVE_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_INTAKE_SPITTING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_GRABBER, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(constants.PIXEL_CYCLE_LIFT_DOWN, constants.PIXEL_CYCLE_STATE_NOT_STARTED);
                    stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_DRIVER_INPUT);
                } else if(!((String) stateMap.get(constants.PIXEL_CYCLE)).equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
                    stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
                }
                if (toggleMap.get(GAMEPAD_1_A_STATE)) {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_HEIGHT));
                    stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
                    robot.lift.LIFT_IDLE_STATE_POSITION = 200;
                    stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
                } else {
                    if(((String)stateMap.get(robot.fulcrum.FULCRUM_SYSTEM_NAME)).equals(robot.fulcrum.FULCRUM_DOWN)){
                        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_IDLE_STATE);
                    } else if(((String)stateMap.get(robot.fulcrum.FULCRUM_SYSTEM_NAME)).equals(robot.fulcrum.FULCRUM_UP) && ((String)stateMap.get(constants.PIXEL_CYCLE)).equals(constants.PIXEL_CYCLE_STATE_NOT_STARTED)){
                        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
                    }
                    if(gamepad1.a){
                        retractionInProgress = true;
                        retractionTime.reset();
                    }
                    rightButtonCounter = 0;
                }


                if (liftIsGoingUp) {
                    if (fulcrumMovement.milliseconds() > 700) {
                        stateMap.put(robot.fulcrum.FULCRUM_SYSTEM_NAME, robot.fulcrum.FULCRUM_DOWN);
                        liftIsGoingUp = false;
                    }
                }

                gamepad1rightbutton.update(gamepad1.right_bumper);
                if(gamepad1.right_trigger > 0.5){
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_IDLE_STATE);
                    stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
                }
                if (gamepad1rightbutton.getState()) {
                    rightButtonCounter += 1;
                }

                if (rightButtonCounter == 1) {
                    stateMap.put(robot.grabber.GRABBER_SYSTEM_NAME, robot.grabber.GRABBER_DEPOSIT_1st_PIXEL);
                }

                if(rightButtonCounter == 2){
                    stateMap.put(robot.grabber.GRABBER_SYSTEM_NAME, robot.grabber.GRABBER_DEPOSIT_2nd_PIXEL);
                    retractionInProgress = true;
                    if(gamepad1.right_bumper){
                        retractionTime.reset();
                    }
                    rightButtonCounter = 0;
                }


                if(gamepad2.left_bumper && gamepad2.b){
                    hangingMode = true;
                }

                if(gamepad2.a && !hangingMode){
                    stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_1_PIXEL);
                } else{
                    stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
                }

                if(gamepad2.right_trigger > 0.2 && hangingMode){
                    robot.hanging.rightHanging.setPower(-1.0);
                } else if(gamepad2.right_stick_y > 0.2 && hangingMode){
                    robot.hanging.rightHanging.setPower(1.0);
                } else {
                    robot.hanging.rightHanging.setPower(0);
                }

                if(gamepad2.left_trigger > 0.2 && hangingMode){
                    robot.hanging.leftHanging.setPower(-1.0);
                } else if(gamepad2.left_stick_y > 0.2 && hangingMode){
                    robot.hanging.leftHanging.setPower(1.0);
                } else{
                    robot.hanging.leftHanging.setPower(0);
                }

                if(gamepad2.y){
                    stateMap.put(robot.drone.DRONE_SYSTEM_NAME, robot.drone.DRONE_RELEASED);
                }

                if (retractionInProgress) {
                    if (retractionTime.seconds() > 1) {
                        stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
                    }
                    if (retractionTime.seconds() > 2.0) {
                        toggleMap.put(GAMEPAD_1_A_STATE, false);
                        retractionInProgress = false;
                    }
                }
                gamepad2RightButton.update(gamepad2.right_bumper);
                gamepad2LeftButton.update(gamepad2.left_bumper);
                dpadDown.update(gamepad2.dpad_down);
                dpadUp.update(gamepad2.dpad_up);
                gamepad1xbutton.update(gamepad1.x);
                if(dpadDown.getState()){
                    if(drawbridgeCounter != 1){
                        drawbridgeCounter -= 1;
                    }
                    updateDrawbridge(stateMap, robot);
                }
                if(dpadUp.getState()){
                    drawbridgeCounter += 1;
                    updateDrawbridge(stateMap, robot);
                }
                if (gamepad2RightButton.getState() && !hangingMode) {
                    liftCounter += 1;
                }
                if (gamepad2LeftButton.getState() && !hangingMode) {
                    if(liftCounter != 0){
                        liftCounter -= 1;
                    }
                }
                if(gamepad1xbutton.getState() && ((String)stateMap.get(constants.PIXEL_CYCLE)).equals(constants.PIXEL_CYCLE_STATE_IN_PROGRESS)){
                    stateMap.put(constants.PIXEL_CYCLE_INTAKE_INTAKING, constants.PIXEL_CYCLE_STATE_COMPLETE);
                }

                if(robot.lift.liftMotor1.getCurrentPosition() > 230){
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
                robot.updateSystems();
                updateLift(stateMap, robot);
                telemetry.addData("Robot left hanging power", robot.hanging.leftHanging.getPower());
                telemetry.addData("Robot right hanging power", robot.hanging.rightHanging.getPower());
                telemetry.addData("Left Hanging Position", robot.hanging.leftHanging.getCurrentPosition());
                telemetry.addData("Right hanging Position", robot.hanging.rightHanging.getCurrentPosition());
                telemetry.addData("drawbridge position", drawbridgeCounter);
                telemetry.addData("Lift intake automation", robot.startLiftDown());
                telemetry.addData("Drawbridge commanded position", robot.drawbridge.drawBridge.getPosition());
                telemetry.addData("Hardstop commanded position", robot.drawbridge.hardStop.getPosition());
                telemetry.addData("Arm angle", robot.arm.encoder.getCurrentPositionOriginal());
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
