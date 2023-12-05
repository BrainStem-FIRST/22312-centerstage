package org.firstinspires.ftc.teamcode.AutoClasses;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
//import static org.firstinspires.ftc.teamcode.AutoClasses.AutoConstants.targetDistance;
//import static org.firstinspires.ftc.teamcode.AutoClasses.AutoConstants.vRedBackdrop_Center;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.HashMap;
import java.util.Map;

public abstract class AutoAbstractOpMode extends ActionOpMode {
    AutoConstants constants;

    public abstract Pose2d startPose();
    public abstract Action traj_left(MecanumDrive drive, BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(MecanumDrive drive, BrainSTEMRobotA robot);

    public abstract Alliance alliance();

//    HardwareMap hardwareMap;
    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/
        /**    TODO: Can be moved to brainSTEMRobot class    **/
        /******************************************************/

        // Huskylens initialization (device and Selection of algorithm
        BrainSTEMRobotA robot = new BrainSTEMRobotA(hardwareMap, telemetry);
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array

        // Distance sensors
        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");

        // TODO: Determine Alliance (RED/BLUE) and Orientation (LEFT/RIGHT)
        // Assume RED-LEFT for now

        // Setup possible trajectories
        robot.drive.pose = startPose();

        // Additional variables
        int xDirection = 0;
        int yDirection = 0;
        boolean foundX = false;
        boolean foundY = false;
        boolean foundZ = false;
        int zDirection = 0;
        int error;

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

        // If the prop never seen, set it to Center

        Action trajectory;

        switch (targetTagPos) {
            case 1:
            case 4:
                trajectory = traj_left(robot.drive, robot);
                break;
            case 2:
            case 5:
                telemetry.addLine("it did not select the program yet");
                trajectory = traj_center(robot);
                telemetry.addLine("it did the thing");
                telemetry.update();
                break;
            case 3:
            case 6:
                trajectory = traj_right(robot.drive, robot);
                break;
            default:
                telemetry.addLine("it did not select the program yet");
                trajectory = traj_center(robot);
                telemetry.addLine("it did the thing");
                telemetry.update();
                //if we dont see the prop this will default to center5
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

        while (opModeIsActive()) {
            telemetry.addData("Loop Counter: ", ++loopCounter);
            telemetry.addData("target tag: ", targetTagPos);

            runBlocking(trajectory);

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

            if(targetBlockPos >= 0) {
                error = blocks[targetBlockPos].x - 160;
            }
            else {
                if (blocks.length == 0) {
                    telemetry.addLine("didn't see anything");
                    if (robot.drive.pose.position.y < constants.vRedBackdrop_Center.y) {    //TODO: abstract backdrop
                        error = -160;
                    }
                    else {
                        error = 160;
                    }
                }
                else {
                    telemetry.addData("block seen (not the target): ", blocks[0].id);
                    if (blocks[0].id > targetTagPos) {
                        error = -160;
                    }
                    else {
                        error = 160;
                    }
                }
            }
            telemetry.addData("error", error);


            // direction to turn
            if (Math.abs(distanceRight - distanceLeft) > 5 && distanceLeft < 500 && !foundZ) {
                if (distanceRight > distanceLeft) {
                    zDirection = -1;
                } else {
                    zDirection = 1;
                }
                telemetry.addLine("turning");
                telemetry.addData("distance error", Math.abs(distanceLeft-distanceRight));
            }
            else {
                zDirection = 0;
                telemetry.addLine("stopped turning");
                telemetry.addData("distance error", Math.abs(distanceLeft-distanceRight));
                foundZ = true;
            }

            // which way to strafe?
            if (Math.abs(error) > 20 && !foundY) {
                if(error < 0) {
                    yDirection = -1;
                    telemetry.addLine("strafing right");
                }
                else {
                    yDirection = 1;
                    telemetry.addLine("strafing left");
                }
            }
            else {
//                if (foundZ) {
                    yDirection = 0;
                    foundY = true;
                    telemetry.addLine("stopped strafing");
//                }
            }



            // check if aligned
            distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.MM);
            distanceRight = sensorDistanceRight.getDistance(DistanceUnit.MM);

            telemetry.addData("distance right", distanceRight);
            telemetry.addData("distance left", distanceLeft);


            // Adjust distance from backdrop
            // Only approach to the backdrop if both Y and Z axes were found.
            if (Math.abs(distanceRight - constants.targetDistance) > 10 && !foundX && foundY && foundZ) {
                if (distanceRight < constants.targetDistance) {
                    xDirection = 1;
                    telemetry.addLine("moving away");
                } else {
                    xDirection = -1;
                    telemetry.addLine("moving towards");
                }
            }
            else {
                if (foundZ && foundY) {
                    xDirection = 0;
                    foundX = true;
                    telemetry.addLine("stop moving");
                }
            }


            // Strafe left or right to approach to the target tag
//            robot.drive.setDrivePowers(new PoseVelocity2d(
//                    new Vector2d(
//                            0.4 * xDirection,
//                            0.35 * yDirection
//                    ),
//                    0.65 * zDirection
//            ));

            if (foundZ) {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0.4 * xDirection,
                                0.4 * yDirection
                        ),
                        0.0
                ));
            }

            robot.drive.updatePoseEstimate();

            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.toDouble()));

            telemetry.addData("lift encoder ticks", robot.lift.liftMotor1.getCurrentPosition());

            telemetry.update();

            runBlocking(new SequentialAction(
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            robot.grabber.setPower(0.8);
                            return false;
                        }
                    },
                    new SleepAction(0.2),
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            robot.lift.raiseHeightTo(robot.lift.LIFT_LOW_STATE_POSITION);

                            // This action will be repeatedly called as long as it returns True.
                            // Return False once the lift is in desired position.
                            return false;
                        }
                    },
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            robot.grabber.setPower(0.0);
                            return false;
                        }
                    },
                    new SleepAction(0.5),
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            robot.arm.armToDepositPosition();
                            return false;
                        }
                    },
                    new SleepAction(0.5),
                    new Action() {
                        @Override
                        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                            robot.grabber.setPower(-1.0);
                            return false;
                        }
                    }
            ));
        }
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
