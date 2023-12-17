package org.firstinspires.ftc.teamcode.AutoClasses;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.robot.StickyButton;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutoAbstractOpMode extends ActionOpMode {
    AutoConstants constants;

    public abstract Pose2d startPose();

    public abstract Action traj_left(BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(BrainSTEMRobotA robot);

    public abstract Action parking_traj(BrainSTEMRobotA robot);

    public abstract Alliance alliance();
    public abstract Orientation orientation();

    private boolean timeDelayIsSet = false;
    private boolean programConfirmation = false;
    private int     autoTimeDelay = 0;

    private StickyButton gamepad1dpadUp = new StickyButton();
    private StickyButton gamepad1dpadDown = new StickyButton();

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

        // Setup possible trajectories
        robot.drive.pose = startPose();

        // Additional variables
        int xDirection = 0;
        int yDirection = 0;
        int zDirection = 0;
        boolean foundX = false;
        boolean foundY = false;
        boolean foundZ = false;
        int position_error;

        double distanceLeft = 0;
        double distanceRight = 0;

        /******** SET THE AUTO TIME DELAY DURING INITIALIZATION *********/
        while (!programConfirmation && !isStopRequested()) {
            setTimeDelay();
        }

        /******** READ PROP POSITION CONTINUOUSLY UNTIL START *********/

        // Determine the prop position
        int targetTagPos = -1;
        int targetBlockPos = -1; // The block of interest within the blocks array.

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

                        if (blocks[i].id == 2) {
                            targetTagPos = getTargetTag(blocks, alliance());
                            telemetry.addData("Found target prop: ", targetTagPos);
                        }
                    }
                } else if (alliance() == Alliance.BLUE) {
                    for (int i = 0; i < blocks.length; i++) {
                        telemetry.addData("Block", blocks[i].toString());

                        if (blocks[i].id == 1) {
                            targetTagPos = getTargetTag(blocks, alliance());
                            telemetry.addData("Found target prop: ", targetTagPos);
                        }
                    }
                }
            } else {
                telemetry.addLine("Don't see the prop :(");

                if (targetTagPos == -1) {
                    telemetry.addLine("(The prop has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the prop before");
                    telemetry.addData("which was: ", targetTagPos);
                }

                sleep(20);
            }
            telemetry.update();
        }

        ////////////// Start is given ///////////////

        /********* CHOOSE YOUR TRAJECTORY BASED ON PROP POSITION ***********/

        Action trajectory;

        switch (targetTagPos) {
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
                telemetry.addLine("it did not select the program yet");
                trajectory = traj_center(robot);
                telemetry.addLine("running default: Center");
                telemetry.update();
                //if we don't see the prop this will default to center
                if (alliance() == Alliance.RED) {
                    targetTagPos = 5;
                }
                else {
                    targetTagPos = 2;
                }
                break;
        }

        // not necessary to call this - start is already given
        // waitForStart();

        // Change recognition mode to AprilTags before the While Loop
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(100);

        int loopCounter = 0;


        /***************   INITIAL TRAJECTORY RUN  *****************/
        /* This was moved outside of the While loop                */
        /***********************************************************/

        telemetry.addData("target tag: ", targetTagPos);
        telemetry.addLine("Started trajectory");
        telemetry.update();

// Disabled to test if it had a negative impact on placing pixels at the board
        runBlocking(new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.grabber.grabber.setPosition(1.0);
                return false;
            }
        });


        //////////////////////////////////////////////////////////
        //                GO TO BACKDROP
        //////////////////////////////////////////////////////////
//TEMP
        runBlocking(new SequentialAction( // TODO: Should this be inside or outside of While loop? Does it matter?
                new SleepAction(autoTimeDelay), // wait for specified time before running trajectory
                trajectory
        ));

        telemetry.addLine("Finished trajectory");
        telemetry.update();

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

                if (blocks[i].id == targetTagPos) {
                    targetBlockPos = i;
                    telemetry.addData("block seen (target): ", blocks[i].id);
                }
            }

            telemetry.addData("block of interest is in slot", targetBlockPos);

            if (targetBlockPos >= 0) {
                position_error = blocks[targetBlockPos].x - 160;
            } else {
                if (blocks.length == 0) {
                    telemetry.addLine("didn't see anything");
                    if (robot.drive.pose.position.y < constants.vRedBackdrop_Center.y) {
                        position_error = -160;
                    } else {
                        position_error = 160;
                    }
                } else {
                    telemetry.addData("block seen (not the target): ", blocks[0].id);
                    if (blocks[0].id > targetTagPos) {
                        position_error = -160;
                    } else {
                        position_error = 160;
                    }
                }
            }
            telemetry.addData("position error", position_error);


            // Read distance
            distanceLeft = (((DistanceSensor) sensorDistanceLeft).getDistance(DistanceUnit.MM)); //sensorDistanceLeft.getDistance(DistanceUnit.MM);
            distanceRight = (((DistanceSensor) sensorDistanceRight).getDistance(DistanceUnit.MM)); //sensorDistanceRight.getDistance(DistanceUnit.MM);

            telemetry.addData("distance right", distanceRight);
            telemetry.addData("distance left", distanceLeft);


            // direction to turn
            if (distanceLeft < 1000.00 && distanceRight < 1000.00 && !foundZ) { // don't bother turning if at least one sensor doesn't see the board
                if (Math.abs(distanceRight - distanceLeft) > 20.00 ) {
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
                } else {
                    yDirection = 1;
                    telemetry.addLine("strafing right");
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
                    0.25 * zDirection
            ));


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
                }
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


    // Returns the position of the prop.
    // If not recognized, returns CENTER (2 or 5 depending on alliance)
    int getTargetTag(HuskyLens.Block[] blocks, Alliance a) {

        int propPos;
        // for test purposes, return a known value
        // delete this segment when team prop is available
        //        return 1;

        if (blocks.length == 1) {
            if (blocks[0].x < 110) {
                // Prop is on left
                propPos = (a == Alliance.BLUE) ? 1 : 4;
            } else if (blocks[0].x > 210) {
                // prop is on right
                propPos = (a == Alliance.BLUE) ? 3 : 6;
            } else {
                // prop is on center
                propPos = (a == Alliance.BLUE) ? 2 : 5;
            }
        } else {
            // could not recognize; return center
            propPos = (a == Alliance.BLUE) ? 2 : 5;
        }

        return propPos;
    }

    private void setTimeDelay() {
        timeDelayIsSet = false;


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
                programConfirmation = true;
                confirmation = true;
            } else if (gamepad2.b) {
                telemetry.clearAll();
                telemetry.addLine("Program Rejected");
                telemetry.update();
                programConfirmation = false;
                timeDelayIsSet = false;
                confirmation = true;
            }
        }

        sleep(500);
        telemetry.clearAll();
    }

}
