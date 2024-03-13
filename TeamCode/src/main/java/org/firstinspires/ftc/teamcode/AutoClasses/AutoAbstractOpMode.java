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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.StickyButton;

public abstract class AutoAbstractOpMode extends LinearOpMode {
    AutoConstants constants;

    public abstract Pose2d startPose();

    // Trajectories are defined in child classes - different for each auto program
    public abstract Action traj_init(BrainSTEMRobotA robot);
    public abstract Action traj_left(BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(BrainSTEMRobotA robot);
    public abstract Action cycle(BrainSTEMRobotA robot);

    public abstract Action parking_traj(BrainSTEMRobotA robot);

    public abstract Alliance alliance();
    public abstract Orientation orientation();

    // Method is defined in child classes AutoXXSafe and AutoXXCycle child classes
    public abstract Boolean cycleAllowed();


    // Used for setting a time delay before starting Auto
    private int     autoTimeDelay = 0;

    private StickyButton gamepad1dpadUp = new StickyButton();
    private StickyButton gamepad1dpadDown = new StickyButton();

    // Used for exception handling
    private ElapsedTime turnTimer = new ElapsedTime();
    private boolean turnTimerStarted = false;

    private ElapsedTime findSpikeTimer = new ElapsedTime();
    private boolean firstTimeRun = true;

    // SENSOR BYPASS: When set, camera based approach is disabled,
    // except for moving towards the board after completion of the trajectory
    private boolean sensorBypass = true;
    private final boolean cycleAllowed = cycleAllowed();

    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/

        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array

        // Distance sensors
        // Rev 2m Distance Sensor measurement range: 5 to 200cm with 1mm resolution
        // Out of range value is >800
        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Set starting pose since robot was initialized with Pose2d(0,0,0)
        robot.drive.pose = startPose();

        // variables controlling approach to the backdrop using camera and distance sensors
        int xDirection = 0;
        int yDirection = 0;
        int zDirection = 0;
        // boolean foundX = false;
        boolean foundY = false;
        boolean foundZ = false;
        int position_error;


        // Fail safe for Strafe caught up in a loop
        int prevStrafeDir = 1; // randomly assigned
        int strafeCounter = 0; // how many times the strafing changed direction


        double distanceLeft = 0;
        double distanceRight = 0;

        //////////////////////////////////////////////////////////
        //                INITIALIZE BEFORE AUTO                //
        //////////////////////////////////////////////////////////
        robot.depositor.bothDepositorsDeposit();
        robot.drawbridge.setDrawBridgeDown();
        robot.drawbridge.setHardstopPosition(0.01);

        // Any initialization of servos before auto will be done here:
        runBlocking(new SequentialAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_IDLE_STATE_POSITION);
                        return false;
                    }
                }
                // Lift remains in idle (not touching) position to allow picking up white pixel later on.
        ));

        /******** SET THE AUTO TIME DELAY DURING INITIALIZATION *********/

//        setTimeDelay();

        // TODO: Add configurations for Cycle Off, Park center/far, Sensor Bypass

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
        //                GO TO BACKDROP                        //
        //////////////////////////////////////////////////////////

        telemetry.addData("target tag: ", targetAprilTagNum);
        telemetry.addLine("Started trajectory");
        telemetry.update();

        runBlocking(new SequentialAction(
                new SleepAction(autoTimeDelay), // wait for specified time before running trajectory
                traj_init(robot) // all variations first go to center spike
        ));

        robot.drive.updatePoseEstimate(); // This should not be unnecessary since updatePoseEstimate is already being called within findSpike()

        runBlocking(new SequentialAction(
                getTrajectory(robot, targetAprilTagNum),
                telemetryPacket -> {
                    telemetry.addLine("Ending pose:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                }
                // getDepositTrajectory() was moved to after sensor-based alignment
        )); // Need to calculate trajectories dynamically

        telemetry.addLine("Finished trajectory");
        telemetry.update();

        //////////////////////////////////////////////////////////
        //           FINAL APPROACH USING SENSORS               //
        //////////////////////////////////////////////////////////

        // At this point the robot must already be positioned in front of the backdrop across the right AprilTag
        // The rest is for fine adjustments. Tolerances can be adjusted for X, Y, Z axes to determine accuracy.

        int targetBlockPos = -1; // The block of interest within the blocks array.

        // Loop if not bypassed, and until the robot aligned/centered with targeted AprilTag
        // (final approach to backdrop  was taken outside of this loop)
        while (opModeIsActive() && !sensorBypass && !foundY) {

            telemetry.addData("Loop Counter: ", ++loopCounter);

            // TODO: Move the AprilTag read and strafe to a separate method

            blocks = robot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            telemetry.update();

            // Poll all block[i] and check if any of their id matches targetPos
            // That [i] is saved as targetBlockPos to be used as index to the block array

            targetBlockPos = -1;    // This will crash the program if no blocks were seen.
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());

                if (blocks[i].id == targetAprilTagNum) {
                    targetBlockPos = i;
                    telemetry.addData("block seen (target): ", blocks[i].id);
                }
            }

            telemetry.addData("block of interest is in slot", targetBlockPos);


            // Read distance to the backdrop
            distanceLeft = (((DistanceSensor) sensorDistanceLeft).getDistance(DistanceUnit.MM));
            distanceRight = (((DistanceSensor) sensorDistanceRight).getDistance(DistanceUnit.MM));

            telemetry.addData("distance right", distanceRight);
            telemetry.addData("distance left", distanceLeft);

            // Calculate position error if the target AprilTag was seen
            position_error = 0;

            if (targetBlockPos >= 0) {
                position_error = blocks[targetBlockPos].x - 160;    // Huskylens resolution is 320pixel width
            }
            else {  // Camera did not see the correct AprilTag (i.e. targetBlockPos is still -1)
                if (blocks.length == 0) {
                    telemetry.addLine("didn't see anything");
                    if ((robot.drive.pose.position.x + constants.robot_length/2) >
                            (constants.FIELD_BACKDROP_X - constants.minTagViewingDistance/constants.IN_TO_MM)) {
                        position_error = 0;
                        foundY = true;  // Cannot adjust based on camera, so accept your current y position
                        telemetry.addLine("too close to backdrop");
                    }
                    // Robot is in viewing distance but still not seeing any AprilTags or it is too far back and cannot see.
                    else
                        if ((robot.drive.pose.position.x + constants.robot_length/2) >
                                (constants.FIELD_BACKDROP_X - constants.maxTagViewingDistance/constants.IN_TO_MM)) {
                            // Robot is in viewing distance but cannot see any tags. Must be stranded in between the backdrops or lost its heading
                            if (robot.drive.pose.position.y > constants.vRedBackdrop_Center.y ||
                                robot.drive.pose.position.y < constants.vBlueBackdrop_Center.y) {
                                telemetry.addLine("in between backdrops");
                                // Set position error such that the robot moves towards the alliance backdrop
                                position_error = (alliance()==Alliance.RED)?-160:160;
                            }
                            else {  // Robot is where it should be seeing the tags but still can't. Must have lost heading.
                                telemetry.addLine("should be seeing tags but can't. Is heading off?");
                                telemetry.addData("heading:", Math.toDegrees(robot.drive.pose.heading.log()));
                                // TODO: This scenario not implemented. Accept robot's current y position
                                foundY = true;
                            }
                        }
                        else {
                            // Robot is far away, cannot see tags. Bring the robot closer to the backdrop
                            telemetry.addLine("far away from backdrop, cannot see tags.");
                            telemetry.addData("heading:", Math.toDegrees(robot.drive.pose.heading.log()));
                            // TODO: Not implemented. Accept robot's current y position
                            foundY = true;
                        }
                }
                else { // Seen some AprilTags but not the one targeted. It means robot's y position is off
                    telemetry.addData("block seen (not the target): ", blocks[0].id);
                    if (blocks[0].id > targetAprilTagNum) {
                        position_error = -160;
                    } else {
                        position_error = 160;
                    }
                }
            }
            telemetry.addData("position error", position_error);

            // direction to turn
            if (distanceLeft < constants.maxTagViewingDistance && distanceRight < constants.maxTagViewingDistance && !foundZ) { // don't bother turning if at least one sensor doesn't see the board
                if (Math.abs(distanceRight - distanceLeft) > 8.00){
                    if (distanceRight > distanceLeft) {
                        zDirection = -1;
                    } else {
                        zDirection = 1;
                    }
                    telemetry.addLine("turning");
                    telemetry.addData("turn error", Math.abs(distanceLeft - distanceRight));
                } else {
                    // heading is within tolerance
                    zDirection = 0;
                    foundZ = true;
                    telemetry.addLine("stopped turning");
                    telemetry.addData("turn error", Math.abs(distanceLeft - distanceRight));
                }
            } else {
                zDirection = 0;
                telemetry.addLine("turning paused due to OOR sensor, or heading is within tolerance.");
                telemetry.addData("turn error", Math.abs(distanceLeft - distanceRight));

                // Give up after 2 seconds
                if (!turnTimerStarted) {
                    turnTimer.reset();
                    telemetry.addLine("Turning time started!");
                    turnTimerStarted = true;
                }

                if (turnTimer.seconds() > 2.0) {
                    foundZ = true;
                    zDirection = 0;
                    telemetry.addLine("Turning timed out");
                }
            }

            // which way to strafe?
            if (Math.abs(position_error) > 8 && !foundY) {
                if (position_error < 0) {
                    yDirection = -1;
                    telemetry.addLine("strafing left");

                    // increment counter if the direction changed
                    if (prevStrafeDir == 1) {
                        strafeCounter++;
                        prevStrafeDir = -1;
                    }
                } else {
                    yDirection = 1;
                    telemetry.addLine("strafing right");

                    // increment counter if the direction changed
                    if (prevStrafeDir == -1) {
                        strafeCounter++;
                        prevStrafeDir = 1;
                    }
                }

                // Exit strafing if it caught up in a loop
                if (strafeCounter > 3) {
                    yDirection = 0;
                    foundY = true;
                    telemetry.addLine("Exit strafing due to frequent direction change.");
                }

                // TODO: if position_error is 160 or -160 more than x seconds, do something
            } else {
                yDirection = 0;
                telemetry.addLine("stopped strafing temporarily");

                if (foundZ) {  // do not stop seeking the tag unless turning is complete. turning can make you lose position.
                    foundY = true;
                    telemetry.addLine("stopped strafing permanently");
                }
            }

            // Strafe left or right to approach to the target tag
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.27 * xDirection,
                            0.32 * yDirection
                    ),
                    0.2 * zDirection
            ));


            robot.drive.updatePoseEstimate();

            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            telemetry.update();
        }

        /////////////////////////////////////////////////////////////////
        //
        //  If sensors used, robot is now aligned with the AprilTag.
        //  If sensors bypassed, robot is at the end of its center/left/right trajectory.
        //  Either case, start the depositor action which will move (strafe) the robot towards
        //  the backdrop by an amount calculated from the distance sensors.
        //
        /////////////////////////////////////////////////////////////////


        //////////////////////////////////////////////////////////
        //
        // Deposit yellow pixel(s)
        //
        //////////////////////////////////////////////////////////
        runBlocking(getDepositTrajectory(robot, targetAprilTagNum, getDistanceToBackdrop(sensorDistanceLeft,sensorDistanceRight)));
        telemetry.update();


        //////////////////////////////////////////////////////////
        //
        // After yellow pixel (end of safe auto), run cycle trajectories
        //
        //////////////////////////////////////////////////////////
        if(cycleAllowed) {
            runBlocking(new SequentialAction(
                    cycle(robot),
                    getDepositTrajectory(robot, targetAprilTagNum, getDistanceToBackdrop(sensorDistanceLeft,sensorDistanceRight))
            ));
        }

        //////////////////////////////////////////////////////////
        //
        // Go to park
        //
        //////////////////////////////////////////////////////////
        runBlocking(new SequentialAction(
                parking_traj(robot),
                robot.arm.armToIdle,
                robot.lift.lowerLiftToIdleState
        ));

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


        boolean confirmation = false;
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
                trajectory = traj_center(robot);
                telemetry.addLine("running default: Right");
                telemetry.update();
                break;
        }

        return trajectory;
    }

    // Returns the trajectory with actions to deposit pixels
    // Deposit trajectories moves the robot in x direction by distToBackdrop
    // unless the distToBackdrop is invalid, in which case it moves it to x=52
    public Action getDepositTrajectory(BrainSTEMRobotA robot, int targetTagNum, double distToBackdrop) {
        double x, y;
        Action traj_deposit;

        robot.drive.updatePoseEstimate();

        // assign a default x position if the received distance value is invalid
        if(distToBackdrop<0) {
            x = 52;
        }
        else {
            x = robot.drive.pose.position.x + distToBackdrop;
        }
        y = robot.drive.pose.position.y;

        switch (targetTagNum) {
            case 1:
            case 4:
                // Depositing to left side; turn wrist to 0deg
                traj_deposit = robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
                        .afterTime(0, robot.lift.raiseLiftAutoToLowState)
                        .afterTime(0.5, robot.arm.armToDeposit)
                        .afterTime(0.7, robot.wrist.turnWristZero)

                        .strafeToConstantHeading(new Vector2d(x, y))
                        .afterTime(0,robot.depositor.bothDepositorsDeposit)
                        .build();
                break;
            case 2:
            case 5:
            case 3:
            case 6:
                // Depositing to center side; turn wrist to 180deg
                traj_deposit = robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
                        .afterTime(0, robot.lift.raiseLiftAutoToLowState)
                        .afterTime(0.5, robot.arm.armToDeposit)
                        .afterTime(0.7, robot.wrist.turnWristOneEighty)

                        .strafeToConstantHeading(new Vector2d(x, y))
                        .afterTime(0,robot.depositor.bothDepositorsDeposit)
                        .build();
                break;
            default:
                // This default should never be reached because a default value for
                // targetTagPos is already assigned during readPropPosition().
                // Still...
                telemetry.addLine("BUG IN CODE! Target Tag Number was not properly set.");
                telemetry.addLine("running default: Right");
                telemetry.update();
                // Depositing to right side; turn wrist to 180deg
                traj_deposit = robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
                        .afterTime(0, robot.lift.raiseLiftAutoToLowState)
                        .afterTime(0.5, robot.arm.armToDeposit)
                        .afterTime(0.7, robot.wrist.turnWristOneEighty)

                        .strafeToConstantHeading(new Vector2d(x, y))
                        .afterTime(0,robot.depositor.bothDepositorsDeposit)
                        .build();
                break;
        }

        return traj_deposit;
    }


    ///////////////////////////////////////////////////////////////
    //
    // Continuously read the scene and determine if RED or BLUE
    // prop was detected. Then call getTargetTag() for the actual
    // april tag to look for (based on where the prop is located).
    //
    // If no prop is detected, return a default target tag number.
    //
    ///////////////////////////////////////////////////////////////
    private int readPropPosition(BrainSTEMRobotA robot) {
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
            targetTagNum = (alliance()==Alliance.BLUE) ? 2 : 5;
        }

        return targetTagNum;
    }

    // Returns the position of the prop based on block's relative location on display.
    // Must receive a valid block (i.e. not null)
    private int getTargetTag(HuskyLens.Block block) {

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


    ///////////////////////////////////////////////////////////////
    //
    // Calculates the distance the robot should travel in x axis
    // from its current position before getting to the deposit
    // distance (practically touching the backdrop)
    //
    ///////////////////////////////////////////////////////////////
    private double getDistanceToBackdrop(DistanceSensor sensorDistanceLeft, DistanceSensor sensorDistanceRight) {
        // Read the distance sensor again
        double distanceLeft = (((DistanceSensor) sensorDistanceLeft).getDistance(DistanceUnit.MM));
        double distanceRight = (((DistanceSensor) sensorDistanceRight).getDistance(DistanceUnit.MM));

        // Determine the distance to backdrop in inches
        double distToBackdropHypo;
        double distToBackdrop;

        // Check validity of sensor readings
        if (distanceRight < 500) // Right sensor is valid, use it
            distToBackdropHypo = distanceRight;
        else if (distanceLeft < 500) // Right sensor is OOR, left sensor is valid, use it
            distToBackdropHypo = distanceLeft;
        else
            distToBackdropHypo = -1; // Neither distance sensors are working, mark distToBackdrop invalid

        if (distToBackdropHypo < 0)
            distToBackdrop = -1;
        else {
            double c = distToBackdropHypo - constants.targetDistance;
            double b = constants.distSensorHeight - (constants.targetDistance * Math.sin(Math.toRadians(20)));
            distToBackdrop = Math.sqrt(Math.pow(c,2) - Math.pow(b,2)) / constants.IN_TO_MM;
        }

        telemetry.addData("Distance to move towards backdrop: ", distToBackdrop);

        return distToBackdrop;
    }

}
