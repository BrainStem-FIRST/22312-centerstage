package org.firstinspires.ftc.teamcode.AutoClasses;


import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.StickyButton;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // Depositing on the backdrop is common to all auto programs but may differ
    // depending on which side of the board the yellow pixel (or white pixels)
    // will be deposited (i.e. which way the wrist will turn may be different
//    public Action deposit_right(BrainSTEMRobotA robot);
//    public Action deposit_center(BrainSTEMRobotA robot);
//    public Action deposit_left(BrainSTEMRobotA robot);

    // Used for setting a time delay before starting Auto
    private int     autoTimeDelay = 0;

    private StickyButton gamepad1dpadUp = new StickyButton();
    private StickyButton gamepad1dpadDown = new StickyButton();

    // Used for exception handling
    private ElapsedTime turnTimer = new ElapsedTime();
    private boolean turnTimerStarted = false;

    private ElapsedTime findSpikeTimer = new ElapsedTime();
    private boolean firstTimeRun = true;



    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/

        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);
        robot.depositor.grabBothPixels();
//        robot.wrist.wristToPickUpPosition();
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

        // Any initialization of servos before auto will be done here:
        runBlocking(new SequentialAction(
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_GROUND_STATE_POSITION);
                        return false;
                    }
                },
                new ParallelAction(
                        new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                robot.depositor.grabBothPixels();
                                return false;
                            }
                        },
                        robot.drawbridge.drawBridgeDown

// Remove this commented section if the equivalent drawBridgeDown works.
//                        new Action() {
//                            @Override
//                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                                robot.drawbridge.setDrawBridgeDown();
//                                return false;
//                            }
//                        }
                    ),
                new SleepAction(0.5)
        ));



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
                getTrajectory(robot, targetAprilTagNum)
//                getDepositTrajectory(robot, targetAprilTagNum) TODO: delete if my thing works
        )); // Need to calculate trajectories dynamically

        telemetry.addLine("Finished trajectory");
        telemetry.update();

        //////////////////////////////////////////////////////////
        //           FINAL APPROACH USING SENSORS               //
        //////////////////////////////////////////////////////////

        // At this point the robot must already be positioned in front of the backdrop across the right AprilTag
        // The rest is for fine adjustments. Tolerances can be adjusted for X, Y, Z axes to determine accuracy.

        int targetBlockPos = -1; // The block of interest within the blocks array.

        while (opModeIsActive() && !foundX) { // exit the loop once the robot aligned/centered and finally approached

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
                    if (distanceRight < constants.minTagViewingDistance) {
                        position_error = 0;
                        foundY = true;  // Cannot adjust based on camera, so accept your current y position
                        telemetry.addLine("too close to backdrop");
                    }
                    // Robot is in viewing distance but still not seeing any AprilTags or it is too far back and cannot see.
                    else
                        if (distanceRight < constants.maxTagViewingDistance) {
                            // Robot is in viewing distance but cannot see any tags. Must be stranded in between the backdrops or lost its heading
                            if (robot.drive.pose.position.y > constants.vRedBackdrop_Center.y ||
                                robot.drive.pose.position.y < constants.vBlueBackdrop_Center.y) {
                                telemetry.addLine("in between backdrops");
                                // Set position error such that the robot moves towards the alliance backdrop
                                position_error = (alliance()==Alliance.RED)?-160:160;
                            }
                            else {  // Robot is where it should be seeing the tags but still can't. Must have lost heading.
                                telemetry.addLine("should be seeing tags but can't. Is heading off?");
                                // TODO: This scenario not implemented. Accept robot's current y position
                                foundY = true;
                            }
                        }
                        else {
                            // Robot is far away, cannot see tags. Bring the robot closer to the backdrop
                            telemetry.addLine("far away from backdrop, cannot see tags.");
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

            // Adjust distance from backdrop
            // Only approach to the backdrop if both Y and Z axes were found.
            if (foundY) { //foundY also means foundZ
                if (Math.abs(distanceRight - constants.targetDistance) > 8.00 && !foundX) {
                    if (distanceRight < constants.targetDistance) {
                        xDirection = 1;
                        telemetry.addLine("moving away");
                    } else {
                        xDirection = -1;
                        telemetry.addLine("moving towards");
                    }
                } else {
                    xDirection = 0;
                    foundX = true;
                    telemetry.addLine("stopped moving");
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

        // when at backdrop
        runBlocking(new SequentialAction(
                robot.lift.raiseLiftAuto,
                robot.arm.armToDeposit,
                robot.wrist.turnWrist,
                robot.depositor.bothDepositorsDeposit,
                new SleepAction(0.5)
        ));

        // TODO: after yellow pixel--- add subsystem stuff
//        runBlocking(new SequentialAction(
//                cycle(robot)
//        ));

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


        runBlocking(new SequentialAction(
                parking_traj(robot),
                // Reset the subsystems for Tele
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.arm.armToIdlePosition();
                        return false;
                    }
                },

                new SleepAction(1.0),

                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_GROUND_STATE_POSITION);
                        return false;
                    }
                },
                new SleepAction(4.0)


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

    /* TODO: delete if my thing works
    public Action getDepositTrajectory(BrainSTEMRobotA robot, int targetTagNum){
        Action trajectory;
        robot.drive.updatePoseEstimate();
        switch (targetTagNum) {
            case 1:
            case 4:
                trajectory = deposit_left(robot);
                break;
            case 2:
            case 5:
                trajectory = deposit_center(robot);
                break;
            case 3:
            case 6:
                trajectory = deposit_right(robot);
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

     */

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
            targetTagNum = (alliance()==Alliance.BLUE) ? 2 : 5;
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
