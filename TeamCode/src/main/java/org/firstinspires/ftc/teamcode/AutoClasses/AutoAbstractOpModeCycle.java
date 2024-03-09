package org.firstinspires.ftc.teamcode.AutoClasses;


import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.StickyButton;

public abstract class AutoAbstractOpModeCycle extends LinearOpMode {
    AutoConstants constants;

    public abstract Pose2d startPose();

    public abstract Action traj_init(BrainSTEMRobotA robot);
    public abstract Action traj_left(BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(BrainSTEMRobotA robot);

//    public abstract Action deposit_right(BrainSTEMRobotA robot);
//    public abstract Action deposit_center(BrainSTEMRobotA robot);
//    public abstract Action deposit_left(BrainSTEMRobotA robot);
    public abstract Action cycle(BrainSTEMRobotA robot);

    public abstract Action parking_traj(BrainSTEMRobotA robot);

    public abstract Alliance alliance();
    public abstract Orientation orientation();

    // Used for setting a time delay before starting Auto
    private int     autoTimeDelay = 0;

    private StickyButton gamepad1dpadUp = new StickyButton();
    private StickyButton gamepad1dpadDown = new StickyButton();

    // Used for exception handling
    private ElapsedTime turnTimer = new ElapsedTime();
    private boolean firstTimeVisiting = true;

    private ElapsedTime findSpikeTimer = new ElapsedTime();
    private boolean firstTimeRun = true;



    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/

        // Huskylens initialization (device and Selection of algorithm
        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);
        robot.depositor.bothDepositorsDeposit();
        robot.drawbridge.setDrawBridgeDown();
//        robot.lift.raiseHeightTo(robot.lift.LIFT_IDLE_STATE_POSITION);
        robot.drawbridge.setHardstopPosition(0.01);
//        robot.arm.armToIdlePosition();
//        robot.wrist.wristToPickUpPosition();
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array

        // Distance sensors
        // Rev 2m Distance Sensor measurement range: 5 to 200cm with 1mm resolution
        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Set starting pose since robot was initialized with Pose2d(0,0,0)
        robot.drive.pose = startPose();

        // variables controlling approach to the backdrop using camera and distance sensors
        int xDirection = 0;
        int yDirection = 0;
        int zDirection = 0;
        boolean foundX = false;
        boolean foundY = false;
        boolean foundZ = false; //false;
        int position_error;


        // Fail safe for Strafe caught up in a loop
        int prevStrafeDir = 1; // randomly assigned
        int strafeCounter = 0; // how many times the strafing changed direction


        double distanceLeft = 0;
        double distanceRight = 0;

        /******** SET THE AUTO TIME DELAY DURING INITIALIZATION *********/

//        setTimeDelay();

        /******** READ PROP POSITION CONTINUOUSLY UNTIL START *********/

        int targetAprilTagNum = readPropPosition(robot);

        /////////////////////////////////////////////
        //             START WAS GIVEN             //
        //        No need for waitforstart()       //
        /////////////////////////////////////////////

        // Change recognition mode to AprilTags before the While Loop
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        int loopCounter = 0;    // for debug purposes, no longer necessary


        //////////////////////////////////////////////////////////
        //                INITIALIZE BEFORE AUTO                //
        //////////////////////////////////////////////////////////

        telemetry.addData("target tag: ", targetAprilTagNum);
        telemetry.addLine("Started trajectory");
        telemetry.update();

// Any initialization of servos before auto will be done here:
        runBlocking(new SequentialAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_IDLE_STATE_POSITION);
                        return false;
                    }
                }
        ));


        //////////////////////////////////////////////////////////
        //                GO TO BACKDROP                        //
        //////////////////////////////////////////////////////////

        runBlocking(new SequentialAction(
                new SleepAction(autoTimeDelay),
                getTrajectory(robot, targetAprilTagNum)// wait for specified time before running trajectory
        ));

        robot.drive.updatePoseEstimate(); // This should not be unnecessary since updatePoseEstimate is already being called within findSpike()

//        runBlocking(new SequentialAction(
//                getTrajectory(robot, targetAprilTagNum)
//        ));// Need to calculate trajectories dynamically

//        runBlocking(new SequentialAction(
//                getDepositTrajectory(robot, targetAprilTagNum)
//        ));

        telemetry.addLine("Finished trajectory");
        telemetry.update();

        //////////////////////////////////////////////////////////
        //           FINAL APPROACH USING SENSORS               //
        //////////////////////////////////////////////////////////

/*
        // Arrived at position. Place yellow pixel
        runBlocking(new SequentialAction(
                new Action() {  // TODO: This action may not be needed if the grabber is squeezed at the start via golden gear
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.grabber.grabPixel();
                        return false;
                    }
                },
                new SleepAction(0.5),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_LOW_STATE_POSITION);
                        telemetry.addData("lift position", robot.lift.liftMotor2.getCurrentPosition());
                        telemetry.update();
                        return false;
                    }
                },
                new SleepAction(0.4),   // Wait for lift to raise
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.arm.armToDepositPosition();
                        return false;
                    }
                },
                new SleepAction(1.05),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.grabber.depositPixel();
                        return false;
                    }
                }
        ));
*/

/*
        // Reset the lift and arm for the next cycle
        runBlocking(new SequentialAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.arm.armToIdlePosition();
                        return false;
                    }
                },
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_GROUND_STATE_POSITION);
                        return false;
                    }
                }
        ));
*/

        // Run another cycle
//        runBlocking(new SequentialAction(
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        runBlocking(cycle(robot));
//                        return false;
//                    }
//                }
//                // Add actions to place the white pixels
//
//                )
//        );


        // GO TO PARK
        // TODO: In future revisions, add time check to park within 30 seconds


//        runBlocking(new SequentialAction(
//                parking_traj(robot),
//                // Reset the subsystems for Tele
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        robot.arm.armToIdlePosition();
//                        robot.wrist.wristToPickUpPosition();
//                        return false;
//                    }
//                },
//
//                new SleepAction(1.0),
//
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        robot.lift.raiseHeightTo(robot.lift.LIFT_GROUND_STATE_POSITION);
//                        return false;
//                    }
//                },
//                new SleepAction(4.0)
//
//
//        ));

    }



    public enum Alliance {
        RED,
        BLUE
    }

    public enum Orientation {
        LEFT,
        RIGHT
    }


    private void setTimeDelay() {
        boolean timeDelayIsSet = false;

        telemetry.clearAll();

        while (!timeDelayIsSet && !isStopRequested()) {

            gamepad1dpadUp.update(gamepad1.dpad_up);
            gamepad1dpadDown.update(gamepad1.dpad_down);

            telemetry.addLine("Set Time Delay: Driver 1-> DPad UP/DOWN=Increase/Decrease, X=SET.");
            if (gamepad1dpadUp.getState()) {
                telemetry.addLine("dpadUp");
                telemetry.update();
                autoTimeDelay++;
            } else if (gamepad1dpadDown.getState()) {
                autoTimeDelay--;
                if(autoTimeDelay<0) autoTimeDelay=0;
            } else if(gamepad1.x) {
                timeDelayIsSet = true;
            }
            telemetry.addData("delay: ", (int) autoTimeDelay);
            telemetry.update();
        }

        telemetry.clearAll();
        telemetry.addData("Time Delay is Set: ", autoTimeDelay);
        telemetry.update();

        sleep(500);

        telemetry.clearAll();
        telemetry.addLine("Confirm Program:");
        telemetry.addData("Time Delay is Set:", (int) autoTimeDelay);
        telemetry.addData("Alliance:", alliance());
        telemetry.addData("Orientation:", orientation());
        telemetry.addLine("Driver 2-> A To Confirm. B to Restart.");
        telemetry.update();


        boolean confirmation = true;
        while (!confirmation && !isStopRequested()) {
            telemetry.clearAll();
            if (gamepad2.a) {
                telemetry.clearAll();
                telemetry.addLine("Program Confirmed");
                telemetry.update();
                confirmation = true;
            } else if (gamepad2.b) {
                telemetry.clearAll();
                telemetry.addLine("Program Rejected");
                telemetry.update();
                timeDelayIsSet = false;
                confirmation = true;
            }
        }

        sleep(500);
        telemetry.clearAll();
    }

    public Action findSpike(BrainSTEMRobotA robot) {
        return new Action() {

            NormalizedRGBA currentColor;

            int moveToSpike = 0; // move direction
            boolean foundSpike = false;

            final float[] hsv = new float[3];

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                // read current color values
                if(firstTimeRun){
                    findSpikeTimer.reset();
                    firstTimeRun = false;
                }
                currentColor = robot.colorSensor.getNormalizedColors();
                Color.colorToHSV(currentColor.toColor(), hsv);

                telemetry.addLine()
                        .addData("Red", "%.3f", currentColor.red)
                        .addData("Green", "%.3f", currentColor.green)
                        .addData("Blue", "%.3f", currentColor.blue);
                telemetry.addLine()
                        .addData("Hue", "%.3f", hsv[0])
                        .addData("Saturation", "%.3f", hsv[1])
                        .addData("Value", "%.3f", hsv[2]);

                telemetry.addData("Alpha", "%.3f", currentColor.alpha);


/*
                // Find spike based on color value
                if (alliance() == Alliance.RED) {
                    if (currentColor.red > 0.09) { // || robot.drive.pose.position.y > -24) {
                        moveToSpike = 0;
                        foundSpike = true;  // found it
                    } else {
                        moveToSpike = -1;   // keep moving
                    }
                } else {  // Alliance is BLUE
                    if (currentColor.blue > 0.1) { //) || robot.drive.pose.position.y < 24) {
                        moveToSpike = 0;
                        foundSpike = true;
                    } else {
                        moveToSpike = -1;
                    }
                }
 */
                // Find spike based on change in hue value
                // Rubber mat (gray) gives out 180.
                // Any color moves the hue away from 180 (can be in either direction)
                if (Math.abs(148 - hsv[0]) > 30 || findSpikeTimer.seconds() > 0.5){
                    moveToSpike = 0;
                    foundSpike = true;  // found it
                } else {
                    moveToSpike = -1;   // keep moving
                }

                telemetry.addData("found spike:", foundSpike);

                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0.4 * moveToSpike,
                                0.0

                        ),
                        0.0
                ));

                robot.drive.updatePoseEstimate();
                telemetry.addLine("Pose searching findSpike:");
                telemetry.addData("x", robot.drive.pose.position.x);
                telemetry.addData("y", robot.drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                telemetry.update();

                return !foundSpike; // repeat action if not found spike
            }
        };
    }

    // CHOOSE YOUR TRAJECTORY BASED ON PROP POSITION
    // Note 1: Due to dynamic change in pose, the trajectories need to be built on-demand
    //         Call this function after the traj_init (which has movement based on color sensor
    //         is finished and current pose is estimated.
    // Note 2: This method uses public targetTagPos and modifies its value


    private Action getTrajectory(BrainSTEMRobotA robot, int targetTagNum) {
        Action trajectory;
        robot.drive.updatePoseEstimate();

        switch (targetTagNum) {
            case 1:
            case 4:
                trajectory = traj_left(robot);
                break;
            case 2:
            case 5:
                trajectory = traj_center(robot);
                break;
            case 3:
            case 6:
                trajectory = traj_right(robot);
                break;
            default:
                // This default should never be reached because a default value for
                // targetTagPos is already assigned during readPropPosition().
                // Still...
                telemetry.addLine("BUG IN CODE! Target Tag Number was not properly set.");
                trajectory = traj_left(robot);
                telemetry.addLine("running default: Center");
                telemetry.update();
                break;
        }

        return trajectory;
    }

//    public Action getDepositTrajectory(BrainSTEMRobotA robot, int targetTagNum){
//        Action trajectory;
//        robot.drive.updatePoseEstimate();
//        switch (targetTagNum) {
//            case 1:
//            case 4:
//                trajectory = deposit_left(robot);
//                break;
//            case 2:
//            case 5:
//                trajectory = deposit_center(robot);
//                break;
//            case 3:
//            case 6:
//                trajectory = deposit_right(robot);
//                break;
//            default:
//                // This default should never be reached because a default value for
//                // targetTagPos is already assigned during readPropPosition().
//                // Still...
//                telemetry.addLine("BUG IN CODE! Target Tag Number was not properly set.");
//                trajectory = deposit_center(robot);
//                telemetry.addLine("running default: Right");
//                telemetry.update();
//                break;
//        }
//        return trajectory;
//    }

    ///////////////////////////////////////////////////////////////
    //
    // Continuously read the scene and determine if RED or BLUE
    // prop was detected. Then call getTargetTag() for the actual
    // april tag to look for (based on where the prop is located).
    //
    // If no prop is detected, return a default target tag number.
    //
    ///////////////////////////////////////////////////////////////
    int readPropPosition(BrainSTEMRobotA robot) {
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array
        int targetTagNum = -1;

        // find prop and target tag before START
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        while (!isStarted() && !isStopRequested()) {

            // Read the scene
            blocks = robot.huskyLens.blocks();
            telemetry.addData("amount of blocks", blocks.length);

            if (blocks.length != 0) {
                if (alliance() == Alliance.RED) {
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("Block", blocks[i].toString());

                        if (blocks[i].id == 1 || blocks[i].id == 2) {    // Look only for Red
                            targetTagNum = getTargetTag(blocks[i]);
                            telemetry.addData("Found target prop: ", targetTagNum);
                        }
                    }
                } else if (alliance() == Alliance.BLUE) {
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("Block", blocks[i].toString());

                        if (blocks[i].id == 3 || blocks[i].id == 4) {    // Look only for Blue
                            targetTagNum = getTargetTag(blocks[i]);
                            telemetry.addData("Found target prop: ", targetTagNum);
                        }
                    }
                }
            } else {
                telemetry.addLine("Don't see the prop :(");

                if (targetTagNum == -1) {
                    telemetry.addLine("(The prop has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the prop before");
                    telemetry.addData("which was: ", targetTagNum);
                }

                sleep(20);
            }
            telemetry.update();
        } // while

        // return if start is given
        if(targetTagNum == -1) {
            // No prop was detected by the time of Start, return a default value
            // Default is Right
            targetTagNum = (alliance()== Alliance.BLUE) ? 3 : 6; //2 and 5 for default
        }

        return targetTagNum;
    }

    // Returns the position of the prop based on block's relative location on display.
    // Must receive a valid block (i.e. not null)
    int getTargetTag(HuskyLens.Block block) {

        int propPos;
        Alliance a = alliance();

        // for test purposes, return a known value
        // delete this segment when team prop is available
        //        return 1;
        if (block.x < 110) {
            // Prop is on left
            propPos = (a == Alliance.BLUE) ? 1 : 4;
        } else if (block.x > 210) {
            // prop is on right
            propPos = (a == Alliance.BLUE) ? 3 : 6;
        } else {
            // prop is on center
            propPos = (a == Alliance.BLUE) ? 2 : 5;
        }

        return propPos;
    }

}
