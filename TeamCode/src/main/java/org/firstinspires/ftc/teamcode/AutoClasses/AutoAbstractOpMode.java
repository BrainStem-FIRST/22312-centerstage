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

public abstract class AutoAbstractOpMode extends ActionOpMode {
    AutoConstants constants;

    public abstract Pose2d startPose();

    public abstract Action traj_left(BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(BrainSTEMRobotA robot);

    public abstract Action parking_traj(BrainSTEMRobotA robot);

    public abstract Alliance alliance();

    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/
        /**    TODO: Can be moved to brainSTEMRobot class    **/
        /******************************************************/

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

        // TODO: Read blocks continuously until Start. Have some feedback to DS to confirm recognition

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
                targetTagPos = getTargetTag(blocks, alliance());
                telemetry.addData("Found target prop: ", targetTagPos);
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

        Action trajectory;
        Action parkTrajectory;

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
                //if we don't see the prop this will default to center5
                if (alliance() == Alliance.RED) {
                    targetTagPos = 5;
                }
                else {
                    targetTagPos = 2;
                }
                break;
        }


        // Change recognition mode to AprilTags before the While Loop
        robot.huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(100);

        int loopCounter = 0;

        waitForStart();

        while (opModeIsActive() && !foundX) { // exit the loop once the robot aligned/centered and finally approached

            telemetry.addData("Loop Counter: ", ++loopCounter);
            telemetry.addData("target tag: ", targetTagPos);
            telemetry.addLine("Started trajectory");
            telemetry.update();

            runBlocking(trajectory);

            telemetry.addLine("Finished trajectory");
            telemetry.update();

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
            if (distanceLeft < 200.00 || distanceRight < 200.00) { // don't bother turning if at least one sensor doesn't see the board
                if (Math.abs(distanceRight - distanceLeft) > 10.00 && !foundZ) {
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

            // which way to strafe?
            if (Math.abs(position_error) > 20 && !foundY) {
                if (position_error < 0) {
                    yDirection = -1;
                    telemetry.addLine("strafing left");
                } else {
                    yDirection = 1;
                    telemetry.addLine("strafing right");
                }
            } else {
                yDirection = 0;
                telemetry.addLine("stopped strafing");

                if (foundZ) {  // do not stop seeking the tag unless turning is complete. turning can make you lose position.
                    foundY = true;
                    telemetry.addLine("stopped strafing");
                }
            }

            // Adjust distance from backdrop
            // Only approach to the backdrop if both Y and Z axes were found.
            if (foundY && foundZ) {
                if (Math.abs(distanceRight - constants.targetDistance) > 18.00 && !foundX) {
                    if (distanceRight < constants.targetDistance) {
                        xDirection = -1;
                        telemetry.addLine("moving away");
                    } else {
                        xDirection = 1;
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
                            0.3 * xDirection,
                            0.3 * yDirection
                    ),
                    0.45 * zDirection
            ));


            robot.drive.updatePoseEstimate();

            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            telemetry.update();
        }

        // Arrived at position. Place pixel
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
                new SleepAction(0.85),
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        robot.grabber.depositPixel();
                        return false;
                    }
                },
                new SleepAction(0.5),

                parking_traj(robot)
        ));

    }

    public enum Alliance {
        RED,
        BLUE
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

}
