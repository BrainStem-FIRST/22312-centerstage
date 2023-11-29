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
    private boolean retractionInProgress = false;
    private boolean liftIsGoingUp = false;
    private ElapsedTime fulcrumMovement = new ElapsedTime();
    private ElapsedTime retractionTime = new ElapsedTime();
    private final String GAMEPAD_1_A_STATE = "GAMEPAD_1_A_STATE";
    private final String GAMEPAD_1_A_IS_PRESSED = "GAMEPAD_1_A_IS_PRESSED";

    private StickyButton gamepad2RightButton = new StickyButton();
    private StickyButton gamepad2LeftButton = new StickyButton();
    private StickyButton gamepad1RightButton = new StickyButton();
    Map<String, Boolean> toggleMap = new HashMap<String, Boolean>() {{
            put(GAMEPAD_1_A_STATE, false);
            put(GAMEPAD_1_A_IS_PRESSED, false);
        }};
    @Override
    public void runOpMode() {
        Constants constants = new Constants();
        Map<String, String> stateMap = new HashMap<String, String>() {{ }};
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, stateMap);
        stateMap.put(robot.intake.INTAKE_SYSTEM_NAME, robot.intake.INTAKE_IDLE_STATE);
        stateMap.put(robot.hopper.HOPPER_SYSTEM_NAME, robot.hopper.HOPPER_NO_PIXELS);
        stateMap.put(robot.fulcrum.FULCRUM_SYSTEM_NAME, robot.fulcrum.FULCRUM_DOWN);
        stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_2_PIXELS);
        stateMap.put(robot.arm.ARM_SYSTEM_NAME,robot.arm.ARM_IDLE_STATE);
        stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_GROUND_STATE);
        stateMap.put(robot.drawbridge.DRAWBRIDGE_SYSTEM_NAME,robot.drawbridge.DRAWBRIDGE_UP_STATE);
        stateMap.put(robot.grabber.GRABBER_SYSTEM_NAME, robot.grabber.GRABBER_MIDDLE_STATE);
        stateMap.put(constants.DRIVER_2_SELECTED_HEIGHT, robot.lift.LIFT_ROW1_STATE);
//        stateMap.put(robot.hanging.HANGING_SYSTEM_NAME, robot.hanging.SERVO_NOT_RELEASED);
        waitForStart();

        while(!isStopRequested()){
            if(gamepad2.right_bumper && gamepad2.left_bumper){
                robot.lift.resetEncoders();
            } else if(gamepad2.left_trigger > 0.5){
                robot.lift.liftMotor1.setPower(-1.0);
            } else {
                setButtons();
                telemetry.addLine("IN Tele");
                if(gamepad1.right_trigger > 0.5){
                    stateMap.put(constants.PIXEL_CYCLE, constants.PIXEL_CYCLE_STATE_IN_PROGRESS);
                }
                if(gamepad1.left_trigger > 0.5){
                    robot.intake.intakeMotor.setPower(gamepad1.left_trigger);
                }
                if(gamepad2.a){
                    stateMap.put(constants.NUMBER_OF_PIXELS, constants.PIXEL_PICKUP_1_PIXEL);
                }
                if(toggleMap.get(GAMEPAD_1_A_STATE)){
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, stateMap.get(constants.DRIVER_2_SELECTED_HEIGHT));
                    stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_DEPOSIT_STATE);
                    robot.lift.LIFT_IDLE_STATE_POSITION = 160;
                    liftIsGoingUp = true;
                    if(gamepad1.a){
                        fulcrumMovement.reset();
                    }
                } else {
                    stateMap.put(robot.lift.LIFT_SYSTEM_NAME, robot.lift.LIFT_IDLE_STATE);
                    rightButtonCounter = 0;
                }

                if(liftIsGoingUp){
                    if(fulcrumMovement.milliseconds() > 700){
                        stateMap.put(robot.fulcrum.FULCRUM_SYSTEM_NAME, robot.fulcrum.FULCRUM_DOWN);
                        liftIsGoingUp = false;
                    }
                }

                gamepad1RightButton.update(gamepad1.right_bumper);
                if(gamepad1RightButton.getState()){
                    rightButtonCounter += 1;
                }
                if(rightButtonCounter == 2){
                    stateMap.put(robot.grabber.GRABBER_SYSTEM_NAME, robot.grabber.GRABBER_DEPOSIT_2nd_PIXEL);
                    retractionTime.reset();
                    retractionInProgress = true;
                    rightButtonCounter = 0;
                }

                if(rightButtonCounter == 1){
                    stateMap.put(robot.grabber.GRABBER_SYSTEM_NAME, robot.grabber.GRABBER_DEPOSIT_1st_PIXEL);
                }

                if(retractionInProgress){
                    if(retractionTime.seconds() > 1){
                        stateMap.put(robot.arm.ARM_SYSTEM_NAME, robot.arm.ARM_IDLE_STATE);
                    }
                    if(retractionTime.seconds() > 2.0){
                        toggleMap.put(GAMEPAD_1_A_STATE, false);
                        retractionInProgress = false;
                    }
                }
                gamepad2RightButton.update(gamepad2.right_bumper);
                gamepad2LeftButton.update(gamepad2.left_bumper);
                if(gamepad2RightButton.getState()){
                    liftCounter += 1;
                }
                if(gamepad2LeftButton.getState()){
                    liftCounter -= 1;
                }
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x * 0.75));

                robot.drive.updatePoseEstimate();
                robot.updateSystems();
                updateLift(stateMap, robot);
                telemetry.addData("Retraction in progress", retractionInProgress);
                telemetry.addData("Lift encoders", robot.lift.liftMotor1.getCurrentPosition());
                telemetry.addData("Lift counter for driver 2 height", liftCounter);
                telemetry.addData("Right button pushes", rightButtonCounter);
                telemetry.addData("Retraction in progress", retractionInProgress);
                telemetry.addData("Retraction timing", retractionTime.seconds());
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
}
