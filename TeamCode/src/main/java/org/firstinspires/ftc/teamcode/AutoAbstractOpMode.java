package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class AutoAbstractOpMode extends AutoConstants{

    public abstract Pose2d startPose();
    public abstract Action traj_left(MecanumDrive drive);
    public abstract Action traj_center(MecanumDrive drive);
    public abstract Action traj_right(MecanumDrive drive);

    HuskyLens huskyLens;
    HardwareMap hardwareMap;

    public void runOpMode() {

        /************** Hardware Initialization ***************/
        /**    TODO: Can be moved to brainSTEMRobot class    **/
        /******************************************************/

        // Huskylens initialization (device and Selection of algorithm)
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array

        // Drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Distance sensors
        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "sensor_distanceLeft");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "sensor_distanceRight");


        // TODO: Determine Alliance (RED/BLUE) and Orientation (LEFT/RIGHT)
        // Assume RED-LEFT for now

        // Setup possible trajectories
        Pose2d startingPose = pStartingPose_RedLeft;
        drive.pose = startingPose;

        Action reverseTest =
                drive.actionBuilder(startingPose)
                        .setReversed(true)
                        .lineToY(-48)
                        .setReversed(false)
                        .lineToY(-60)
                        .build();

//        Action turnTest =
//                drive.actionBuilder(startingPose)
//                        .setReversed(true)
//                        .lineToY(-48)
//                        .turn(Math.toRadians(0))
//                        .build();

        // Define trajectories for each target position
        Action traj_center =
                drive.actionBuilder(startingPose)
                        // go backwards
                        .setReversed(true)

                        // Replace prop with your yellow pixel (just push)
                        .lineToY(vRedRightSpike_Center.y + robot_length / 2)

                        .stopAndAdd(new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Spit out yellow pixel on center spike.");
                                telemetry.update();
                                return false;
                            }
                        })
                        .endTrajectory()
                        .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                        // Go to backdrop to place your purple pixel
                        .splineTo(vRedClearStageGate, Math.toRadians(0))      // First clear the trusses
                        .splineTo(vRedBackdrop_Center, Math.toRadians(0))     // Then, go to designated tag position

                        .build();

        Action traj_left =
                drive.actionBuilder(startingPose)
                        // go backwards
                        .setReversed(true)

                        .splineTo(vRedLeftSpike_Left, Math.toRadians(90))
                        .lineToY(vRedLeftSpike_Left.y + robot_length / 2)

                        .stopAndAdd(new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Spit out yellow pixel at left spike.");
                                telemetry.update();
                                return false;
                            }
                        })

                        .endTrajectory()
                        .setReversed(true)

                        // Go to backdrop to place your purple pixel
                        .splineTo(vRedClearStageGate, Math.toRadians(0))     // First clear the trusses
                        .splineTo(vRedBackdrop_Left, Math.toRadians(0))     // Then, go to designated tag position

                        .build();

        Action traj_right =
                drive.actionBuilder(startingPose)
                        // go backwards
                        .setReversed(true)

                        // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                        .lineToYSplineHeading(vRedLeftSpike_Right.y, Math.toRadians(0))
                        .endTrajectory()
                        .lineToX(vRedLeftSpike_Right.x - robot_length / 2)

                        // Drop yellow pixel in position
                        .stopAndAdd(new Action() {
                            @Override
                            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                telemetry.addLine("Spit out yellow pixel at right spike.");
                                telemetry.update();
                                return false;
                            }
                        })

                        // Discontinue trajectory
                        .endTrajectory()
                        .setReversed(true)

                        // Goto Backdrop to place your purple pixel
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(-TILE_CENTER_TO_CENTER, -TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(0))

//                        .setReversed(true)
                        .splineTo(vRedClearStageGate, Math.toRadians(0))
//                        .setReversed(true)// First clear the trusses
                        .splineTo(vRedBackdrop_Right, Math.toRadians(0))

                        .build();


        // Additional variables
        int direction = 1;
        int angleDirection = 1;
        int error;

        double distanceLeft = 0;
        double distanceRight = 0;
        boolean approached = false; // indicator that the robot is close enough to the desired tag
        boolean aligned = false;

        // TODO: Read blocks continuously until Start. Have some feedback to DS to confirm recognition

        // Determine the prop position
        int targetTagPos = -1;
        int targetBlockPos = -1; // The block of interest within the blocks array.

        // find prop and target tag before START
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        sleep(100);

        while (!isStarted() && !isStopRequested()) {

            // Read the scene
            blocks = huskyLens.blocks();
            telemetry.addData("amount of blocks", blocks.length);

            if (blocks.length != 0) {
                targetTagPos = getTargetTag(blocks, AutoRL.Alliance.RED); //TODO: this is just an example, change alliance later
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
                trajectory = traj_left;
                break;
            case 2:
            case 5:
                trajectory = traj_center;
                break;
            case 3:
            case 6:
                trajectory = traj_right;
                break;
            default:
                trajectory = traj_center;
                //if we dont see the prop this will default to center5
                if (alliance == AutoRL.Alliance.RED) {
                    targetTagPos = 5;
                }
                else {
                    targetTagPos = 2;
                }
                break;
        }

//          waitForStart();


        // Change recognition mode to AprilTags before the While Loop
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        sleep(100);

        while (opModeIsActive()) {

            runBlocking(trajectory);


//            runBlocking(turnTest);

            /*****************  Huskylens test  *****************************/

            // 1. Read the blocks
            // 2. Print all blocks as String to telemetry (for debugging purposes)
            // 3. Identify the tag to align to (based on where the team prop is located)
            // 4. Strafe right or left to center the robot right in front of the targeted tag
            // 5. Turn left or right just the right amount to align robot with the backdrop using distance sensors

// TODO: Move the AprilTag read and strafe to a separate method

            int testCounter = 1;

            blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            telemetry.addData("Found in attempt #", testCounter);

//            while (blocks.length == 0) {
//                blocks = huskyLens.blocks(); // if no tags recognized, do not proceed; just restart the while loop and re-read blocks
//                testCounter++;
//                telemetry.addData("Block count", blocks.length);
//                telemetry.addData("Found in attempt #", testCounter);
//            }

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
                    if (drive.pose.position.y < vRedBackdrop_Center.y) {    // applicable for RED backdrop
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
                direction = 1;
            } else if (error < -60){
                direction = -1;
            } else {
                direction = 0;
            }

            // check if aligned
            distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.MM);
            distanceRight = sensorDistanceRight.getDistance(DistanceUnit.MM);

            telemetry.addData("distance right", distanceRight);
            telemetry.addData("distance left", distanceLeft);

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

            // Strafe left or right to approach to the target tag
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            0.0,
                            0.2 * direction
                    ),
                    0.2 * angleDirection
            ));


            drive.updatePoseEstimate();

            telemetry.addData("Approached=", approached);
            telemetry.addData("Aligned   =", aligned);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);


            telemetry.update();

        }
    }


    public enum Alliance {
        RED,
        BLUE
    }


    // Returns the position of the prop.
    // If not recognized, returns CENTER (2 or 5 depending on alliance)
    int getTargetTag(HuskyLens.Block[] blocks, AutoRL.Alliance a) {

        int propPos;
        // for test purposes, return a known value
        // delete this segment when team prop is available
        //        return 1;

        if (blocks.length == 1) {
            if (blocks[0].x < 110) {
                // Prop is on left
                propPos = (a == AutoRL.Alliance.BLUE) ? 1 : 4;
            } else if (blocks[0].x > 210) {
                // prop is on right
                propPos = (a == AutoRL.Alliance.BLUE) ? 3 : 6;
            } else {
                // prop is on center
                propPos = (a == AutoRL.Alliance.BLUE) ? 2 : 5;
            }
        } else {
            // could not recognize; return center
            propPos = (a == AutoRL.Alliance.BLUE) ? 2 : 5;
        }

        return propPos;
    }

    // Return turn angle in radian that is necessary to align the robot with the backdrop using distance sensors
    double getAlignmentAngle(DistanceSensor leftSensor, DistanceSensor rightSensor, Telemetry t) {

        double opposite, adjacent, hypotenuse;
        double left, right; // readings from sensors
        double turnAngle; // in radians

        int turnDirection = 1;
        double epsilon = 0.035; // close enough (2 degrees) to return 0 in radians

        // Read distance sensors and calculate legs of the triangle
        left = leftSensor.getDistance(DistanceUnit.MM);
        right = rightSensor.getDistance(DistanceUnit.MM);

        opposite = Math.abs(left - right);
        adjacent = DISTANCE_BETWEEN_DISTSENSORS * IN_TO_MM;
        hypotenuse = Math.sqrt(opposite * opposite + adjacent * adjacent);

        t.addData("opposite  =", opposite);
        t.addData("adjacent  =", adjacent);
        t.addData("hypotenuse=", hypotenuse);
        t.update();

        // Catch exceptions

        // Both distance sensors must hit the backdrop for this algorithm to work.
        // If opposite is a large number, that indicates one of the sensors is not hitting the backdrop.
        // TODO: In that case, use IMU only to determine the turn angle. Until then, return 0 for test purposes

        if (opposite > hypotenuse) {
            turnAngle = 0.0;
        } else {
            turnDirection = (left > right) ? 1 : -1;
            turnAngle = turnDirection * Math.asin(opposite / hypotenuse);

            // don't bother to turn if the calculated angle is too small
            if (turnAngle < epsilon) turnAngle = 0.00;
        }

        t.addData("turnAngle = ", Math.toDegrees(turnAngle));
        t.update();

        return turnAngle;
    }

}
