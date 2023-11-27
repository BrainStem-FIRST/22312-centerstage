package org.firstinspires.ftc.teamcode.AutoClasses;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;
import static org.firstinspires.ftc.teamcode.AutoClasses.AutoConstants.targetDistance;
import static org.firstinspires.ftc.teamcode.AutoClasses.AutoConstants.vRedBackdrop_Center;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.HashMap;
import java.util.Map;

public abstract class AutoAbstractOpMode extends ActionOpMode {

    public Pose2d startingPose;
    public abstract Action traj_left(MecanumDrive drive, BrainSTEMRobotA robot);
    public abstract Action traj_center(BrainSTEMRobotA robot);
    public abstract Action traj_right(MecanumDrive drive, BrainSTEMRobotA robot);

    public abstract Alliance alliance();

//    HardwareMap hardwareMap;

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
        robot.drive.pose = startingPose;

        // Additional variables
        int xDirection = 1;
        int yDirection = 1;
        int angleDirection = 1;
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
                trajectory = traj_center(robot);
                break;
            case 3:
            case 6:
                trajectory = traj_right(robot.drive, robot);
                break;
            default:
                trajectory = traj_center(robot);
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

        while (opModeIsActive()) {

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

                if (blocks[i].id == targetTagPos) targetBlockPos = i;
            }

            telemetry.addData("block of interest is in slot", targetBlockPos);
            telemetry.addData("target tag pose ", targetTagPos);

            if(targetBlockPos >= 0) {
                error = blocks[targetBlockPos].x - 160;
                telemetry.addData("error", error);
            }
            else {
                if (blocks.length == 0) {
                    telemetry.addLine("didn't see anything");
                    if (robot.drive.pose.position.y < vRedBackdrop_Center.y) {    //TODO: abstract backdrop
                        error = -160;
                    }
                    else {
                        error = 160;
                    }
                }
                else {
                    telemetry.addData("block: ", blocks);
                    if (blocks[0].id > targetTagPos) {
                        error = -160;
                    }
                    else {
                        error = 160;
                    }
                }
            }

            // which way to strafe?
            if (error > 60) {
                yDirection = 1;
            } else if (error < -60){
                yDirection = -1;
            } else {
                yDirection = 0;
            }

            // check if aligned
            distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.MM);
            distanceRight = sensorDistanceRight.getDistance(DistanceUnit.MM);

            telemetry.addData("distance right", distanceRight);
            telemetry.addData("distance left", distanceLeft);

            // direction to turn
            if (Math.abs(distanceRight - distanceLeft) > 30) {
                if (distanceRight > distanceLeft) {
                    angleDirection = 1;
                } else {
                    angleDirection = -1;
                }
            }
            else {
                angleDirection = 0;
            }

            if (Math.abs(distanceRight - targetDistance) > 10) {
                if (distanceRight < targetDistance) {
                    xDirection = -1;
                } else {
                    xDirection = 1;
                }
            }
            else {
                xDirection = 0;
            }

            // Strafe left or right to approach to the target tag
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.2 * xDirection,
                            0.2 * yDirection
                    ),
                    0.2 * angleDirection
            ));


            robot.drive.updatePoseEstimate();

            telemetry.addData("x", robot.drive.pose.position.x);
            telemetry.addData("y", robot.drive.pose.position.y);
            telemetry.addData("heading", robot.drive.pose.heading);


            telemetry.update();

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
