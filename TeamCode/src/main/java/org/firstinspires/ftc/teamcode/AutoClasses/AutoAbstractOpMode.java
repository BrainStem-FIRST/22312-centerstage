package org.firstinspires.ftc.teamcode.AutoClasses;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.StickyButton;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutoAbstractOpMode extends ActionOpMode {
    AutoConstants constants;

    public abstract Pose2d startPose();

    public abstract Action traj_init(BrainSTEMRobotA robot);
    public abstract Action traj_left(BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(BrainSTEMRobotA robot);

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



    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/

        // Huskylens initialization (device and Selection of algorithm
        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array

        // Distance sensors
        // Rev 2m Distance Sensor measurement range: 5 to 200cm with 1mm resolution
        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // Set starting pose since robot was initialized with Pose2d(0,0,0)
        robot.drive.pose = startPose();

        // Additional variables
        int xDirection = 0;
        int yDirection = 0;
        int zDirection = 0;
        boolean foundX = false;
        boolean foundY = false;
        boolean foundZ = false;
        int position_error;


        // Fail safe for Strafe caught up in a loop
        int prevStrafeDir = 1; // randomly assigned
        int strafeCounter = 0; // how many times the strafing changed direction


        double distanceLeft = 0;
        double distanceRight = 0;

        /******** SET THE AUTO TIME DELAY DURING INITIALIZATION *********/
//        while (!programConfirmation && !isStopRequested()) {
//            setTimeDelay();
//        }

        /******** READ PROP POSITION CONTINUOUSLY UNTIL START *********/

        int targetAprilTagNum = readPropPosition(robot);

        /////////////////////////////////////////////
        //             START WAS GIVEN             //
        /////////////////////////////////////////////

        // Change recognition mode to AprilTags before the While Loop
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(100);

        int loopCounter = 0;    // for debug purposes, no longer necessary


        //////////////////////////////////////////////////////////
        //                INITIALIZE BEFORE AUTO                //
        //////////////////////////////////////////////////////////

        telemetry.addData("target tag: ", targetAprilTagNum);
        telemetry.addLine("Started trajectory");
        telemetry.update();

// Any initialization of servos before auto will be done here:
//        runBlocking(new SequentialAction (
//                new Action() {
//                    @Override
//                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                        robot.grabber.grabber.setPosition(0.0);
//                        return false;
//                    }
//                },
//                new SleepAction(0.5)
//        ));


        //////////////////////////////////////////////////////////
        //                GO TO BACKDROP                        //
        //////////////////////////////////////////////////////////

        runBlocking(new SequentialAction(
                new SleepAction(autoTimeDelay), // wait for specified time before running trajectory

                traj_init(robot)   // all variations first go to center spike
        ));

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
        )); // Need to calculate trajectories dynamically



        telemetry.addLine("Finished trajectory");
        telemetry.update();

        //////////////////////////////////////////////////////////
        //           FINAL APPROACH USING SENSORS               //
        //////////////////////////////////////////////////////////
        int targetBlockPos = -1; // The block of interest within the blocks array.

        while (opModeIsActive() && !foundX) { // exit the loop once the robot aligned/centered and finally approached

            telemetry.addData("Loop Counter: ", ++loopCounter);

// TODO: Move the AprilTag read and strafe to a separate method

            blocks = robot.huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            telemetry.update();

            // poll all block[i] and check if any of their id matches targetPos
            // targetBlock = i

            targetBlockPos = -1;    // This will crash the program if no blocks were seen.
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());

                if (blocks[i].id == targetAprilTagNum) {
                    targetBlockPos = i;
                    telemetry.addData("block seen (target): ", blocks[i].id);
                }
            }

            telemetry.addData("block of interest is in slot", targetBlockPos);


            // Read distance
            distanceLeft = (((DistanceSensor) sensorDistanceLeft).getDistance(DistanceUnit.MM)); //sensorDistanceLeft.getDistance(DistanceUnit.MM);
            distanceRight = (((DistanceSensor) sensorDistanceRight).getDistance(DistanceUnit.MM)); //sensorDistanceRight.getDistance(DistanceUnit.MM);

            telemetry.addData("distance right", distanceRight);
            telemetry.addData("distance left", distanceLeft);

            if (targetBlockPos >= 0) {
                position_error = blocks[targetBlockPos].x - 160;
            } else {
                if (blocks.length == 0) {
                    telemetry.addLine("didn't see anything");
                    if (distanceRight < constants.minTagVision) {
                        position_error = 0;
                        foundY = true;
                        telemetry.addLine("too close to board");
                    }
                    else if (robot.drive.pose.position.y < constants.vRedBackdrop_Center.y) {
                        position_error = -160;
                    } else {
                        position_error = 160;
                    }
                } else {
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
            if (distanceLeft < 1000.00 && distanceRight < 1000.00 && !foundZ) { // don't bother turning if at least one sensor doesn't see the board
                if (Math.abs(distanceRight - distanceLeft) > 50.00 ) {
                    if (distanceRight > distanceLeft) {
                        zDirection = -1;
                    } else {
                        zDirection = 1;
                    }
                    telemetry.addLine("turning");
                    telemetry.addData("turn error", Math.abs(distanceLeft - distanceRight));
                } else {
                    zDirection = 0;
                    foundZ = true;
                    telemetry.addLine("stopped turning");
                    telemetry.addData("turn error", Math.abs(distanceLeft - distanceRight));
                }
            }
            else {
                zDirection = 0;
                telemetry.addLine("turning paused due to OOR sensor");
                telemetry.addData("turn error", Math.abs(distanceLeft - distanceRight));

                // Give up after 2 seconds
                if(firstTimeVisiting) {
                    turnTimer.reset();
                    telemetry.addLine("Turning time started!");
                    firstTimeVisiting = false;
                }
                
                if (turnTimer.seconds() > 2.0) {
                    foundZ = true;
                    zDirection = 0;
                    telemetry.addLine("Turning timed out");
                }
            }

            // which way to strafe?
            if (Math.abs(position_error) > 12 && !foundY) {
                if (position_error < 0) {
                    yDirection = -1;
                    telemetry.addLine("strafing left");

                    // increment counter if the direction changed
                    if(prevStrafeDir == 1) {
                        strafeCounter++;
                        prevStrafeDir = -1;
                    }
                } else {
                    yDirection = 1;
                    telemetry.addLine("strafing right");

                    // increment counter if the direction changed
                    if(prevStrafeDir == -1) {
                        strafeCounter++;
                        prevStrafeDir = 1;
                    }
                }

                // Exit strafing if it caught up in a loop
                if (strafeCounter>3) {
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

/*
            // Strafe left or right to approach to the target tag
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.27 * xDirection,
                            0.32 * yDirection
                    ),
                    0.25 * zDirection
            ));
 */

            robot.drive.updatePoseEstimate();

            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            telemetry.update();
        }

        // Arrived at position. Place pixel and park
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
                },
                new SleepAction(0.8),

                // GO TO PARK
                // TODO: In future revisions, add time check to park within 30 seconds
                parking_traj(robot),

                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.lift.raiseHeightTo(robot.lift.LIFT_MIDDLE_STATE_POSITION);
                        return false;
                    }
                },

//                new SleepAction(1.0),

                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.arm.armToIdlePosition();
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

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                // read current color values
                currentColor = robot.colorSensor.getNormalizedColors();
                telemetry.addData("gain", robot.colorSensor.getGain());
                telemetry.addData("alliance", alliance());
                telemetry.addData("current color red:", currentColor.red);
                telemetry.addData("current color green:", currentColor.green);
                telemetry.addData("current color blue:", currentColor.blue);

                if (alliance() == Alliance.RED) {
                    if (currentColor.red > 0.03) { //27) { //75) {
                        moveToSpike = 0;
                        foundSpike = true;  // found it
                    } else {
                        moveToSpike = -1;   // keep moving
                    }
                } else {  // Alliance is BLUE
                    if (currentColor.blue > 0.1) {
                        moveToSpike = 0;
                        foundSpike = true;
                    } else {
                        moveToSpike = -1;
                    }
                }

                telemetry.addData("found spike:", foundSpike);

                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0.25 * moveToSpike,
                                0.0

                        ),
                        0.0
                ));

                robot.drive.updatePoseEstimate();
                telemetry.addData("pose", robot.drive.pose.position.y);
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
    //
    private Action getTrajectory(BrainSTEMRobotA robot, int targetTagNum) {
        Action trajectory;

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
                //
                // Still...
                telemetry.addLine("BUG IN CODE! Target Tag Number was not properly set.");
                trajectory = traj_right(robot);
                telemetry.addLine("running default: Right");
                telemetry.update();
                break;
        }

        return trajectory;
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

                        if (blocks[i].id == 1) {    // Look only for Red
                            targetTagNum = getTargetTag(blocks[i]);
                            telemetry.addData("Found target prop: ", targetTagNum);
                        }
                    }
                } else if (alliance() == Alliance.BLUE) {
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("Block", blocks[i].toString());

                        if (blocks[i].id == 2) {    // Look only for Blue
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
            targetTagNum = (alliance()==Alliance.BLUE) ? 3 : 6;
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
