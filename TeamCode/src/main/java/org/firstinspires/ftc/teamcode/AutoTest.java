package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.opencv.core.Mat;

@Config
@Autonomous (name="Robot: Auto Test", group="Robot")
public class AutoTest extends ActionOpMode {
    HuskyLens huskyLens;

    /********************************************
     *   Constants and configurable parameters
     ********************************************/
    static double IN_TO_MM = 25.4;  // conversion multiplier from inch to millimeter
    static double DISTANCE_BETWEEN_DISTSENSORS = 10;    // distance between two sensors in inches

    // Important field positions

    static double TILE_INSIDE_TO_INSIDE = 22.875;
    static double TILE_CENTER_TO_CENTER = 23.625;
    static double TILE_CENTER_TO_EDGE   = 23.25;
    static double TILE_TEETH            = 0.75;

    static double FIELD_BOUNDARY_FROM_CENTER = 2.0 * TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // White Pixel Stack positions
    public static Vector2d vRedStack_Inner   = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,-FIELD_BOUNDARY_FROM_CENTER + 35.0);
    public static Vector2d vRedStack_Center  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,-FIELD_BOUNDARY_FROM_CENTER + 46.875);
    public static Vector2d vRedStack_Outer   = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,-FIELD_BOUNDARY_FROM_CENTER + 58.75);

    public static Vector2d vBlueStack_Inner  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,FIELD_BOUNDARY_FROM_CENTER - 35.0);
    public static Vector2d vBlueStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,FIELD_BOUNDARY_FROM_CENTER - 46.875);
    public static Vector2d vBlueStack_Outer  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,FIELD_BOUNDARY_FROM_CENTER - 58.75);

    // Spike positions
    public static Vector2d vRedLeftSpike_Left   = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 6.0);
    public static Vector2d vRedLeftSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5);
    public static Vector2d vRedLeftSpike_Right  = new Vector2d(-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 9.0); // Not go to center to avoid hitting the A-Beam

    public static Vector2d vRedRightSpike_Left   = new Vector2d(TILE_TEETH / 2.0 + 0.5,-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);
    public static Vector2d vRedRightSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5);
    public static Vector2d vRedRightSpike_Right  = new Vector2d(TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 6.0);

    public static Vector2d vBlueRightSpike_Left   = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    public static Vector2d vBlueRightSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    public static Vector2d vBlueRightSpike_Right  = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    public static Vector2d vBlueLeftSpike_Left   = new Vector2d(TILE_TEETH / 2.0 + 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    public static Vector2d vBlueLeftSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    public static Vector2d vBlueLeftSpike_Right  = new Vector2d(TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    // Backstage positions
    public static double FIELD_BACKSTAGE_X = 2.0 * TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5;
    public static double FIELD_BACKDROP_X  = TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // Backdrop April Tag Positions
    public static Vector2d vRedBackdrop_Left   = new Vector2d(FIELD_BACKDROP_X,-1.5 * TILE_CENTER_TO_CENTER + 6);
    public static Vector2d vRedBackdrop_Center = new Vector2d(FIELD_BACKDROP_X,-1.5 * TILE_CENTER_TO_CENTER);
    public static Vector2d vRedBackdrop_Right  = new Vector2d(FIELD_BACKDROP_X,-1.5 * TILE_CENTER_TO_CENTER - 6);

    public static Vector2d vBlueBackdrop_Left   = new Vector2d(FIELD_BACKDROP_X,1.5 * TILE_CENTER_TO_CENTER - 6);
    public static Vector2d vBlueBackdrop_Center = new Vector2d(FIELD_BACKDROP_X,1.5 * TILE_CENTER_TO_CENTER);
    public static Vector2d vBlueBackdrop_Right  = new Vector2d(FIELD_BACKDROP_X,1.5 * TILE_CENTER_TO_CENTER + 6);

    // Important waypoints on the field
    public static Vector2d vRedClearStageGate  = new Vector2d(TILE_CENTER_TO_CENTER/2.0,-TILE_CENTER_TO_CENTER/2.0);
    public static Vector2d vBlueClearStageGate  = new Vector2d(TILE_CENTER_TO_CENTER/2.0,TILE_CENTER_TO_CENTER/2.0);

    // Robot dimensions. Will become handy to orient around waypoints
    static double robot_length = 14;
    static double robot_width  = 12;

    // Starting positions
    public static Pose2d pStartingPose_RedLeft   = new Pose2d(-1.5*TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER+robot_length/2, Math.toRadians(-90));
    public static Pose2d pStartingPose_RedRight  = new Pose2d( 0.5*TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER+robot_length/2, Math.toRadians(-90));
    public static Pose2d pStartingPose_BlueRight = new Pose2d(-1.5*TILE_CENTER_TO_CENTER,  FIELD_BOUNDARY_FROM_CENTER-robot_length/2, Math.toRadians(90));
    public static Pose2d pStartingPose_BlueLeft  = new Pose2d( 0.5*TILE_CENTER_TO_CENTER,  FIELD_BOUNDARY_FROM_CENTER-robot_length/2, Math.toRadians(90));



    @Override
    public void runOpMode() {

        /************** Hardware Initialization ***************/
        /**    TODO: Can be moved to brainSTEMRobot class    **/
        /******************************************************/

        // Huskylens initialization (device and Selection of algorithm)
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        HuskyLens.Block[] blocks;   // recognized objects will be added to this array

        // Drivetrain
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Distance sensors
        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft  = hardwareMap.get(DistanceSensor.class, "sensor_distanceLeft");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "sensor_distanceRight");


        // TODO: Determine Alliance (RED/BLUE) and Orientation (LEFT/RIGHT)
        // Assume RED-LEFT for now

        // Setup possible trajectories
        Pose2d startingPose = pStartingPose_RedLeft;
        drive.pose = startingPose;

        // Define trajectories for each target position
        Action traj_center =
            drive.actionBuilder(startingPose)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(vRedRightSpike_Center.y+robot_length/2)

                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetry.addLine("Spit out yellow pixel on center spike.");
                        telemetry.update();
                        return false;
                    }
                })

                // Goto Backdrop to place your purple pixel
                .splineTo(vRedClearStageGate,Math.toRadians(0))     // First clear the trusses
                .splineTo(vRedBackdrop_Center,Math.toRadians(0))     // Then, go to designated tag position

                .build();

        Action traj_left  =
            drive.actionBuilder(startingPose)
                // go backwards
                .setReversed(true)

                .splineTo(vRedLeftSpike_Left,Math.toRadians(90))
                .lineToY(vRedLeftSpike_Left.y+robot_length/2)

                .stopAndAdd(new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetry.addLine("Spit out yellow pixel at left spike.");
                        telemetry.update();
                        return false;
                    }
                })

                // Goto Backdrop to place your purple pixel
                .splineTo(vRedClearStageGate,Math.toRadians(0))     // First clear the trusses
                .splineTo(vRedBackdrop_Left,Math.toRadians(0))     // Then, go to designated tag position

                .build();

        Action traj_right =
            drive.actionBuilder(startingPose)
                // go backwards
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(vRedLeftSpike_Right.y,Math.toRadians(0))
                .endTrajectory()
                .lineToX(vRedLeftSpike_Right.x-robot_length/2)

                // Drop yellow pixel in position
                .stopAndAdd(new Action() {
                     @Override
                     public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                         telemetry.addLine("Spit out yellow pixel at right spike.");
                         telemetry.update();
                         return false;
                     }
                })

                // Discontinue trajectory since angles do not jive
                .endTrajectory()

                // Goto Backdrop to place your purple pixel
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER/2.0,Math.toRadians(180.00001)),Math.toRadians(0))

                .splineTo(vRedClearStageGate,Math.toRadians(0))     // First clear the trusses
                .splineTo(vRedBackdrop_Right,Math.toRadians(0))

                .build();


        // Additional variables
        int direction = 1;
        int error;

        double distanceLeft = 0;
        double distanceRight = 0;
        boolean approached = false; // indicator that the robot is close enough to the desired tag
        boolean aligned = false;

        // TODO: Read blocks continuously until Start. Have some feedback to DS to confirm recognition

        // Read the scene to recognize team prop during init
        blocks = huskyLens.blocks();

        // Determine the prop position
        int targetTagPos = getTargetTag(blocks, alliance.RED);
        int targetBlockPos = -1; // The block of interest within the blocks array.


        waitForStart();

        while (opModeIsActive()) {


            runBlocking(
                new SequentialAction(
                        traj_center,

                        drive.actionBuilder(drive.pose)
                                .turn(getAlignmentAngle(sensorDistanceLeft, sensorDistanceRight, telemetry))
                                .build()
                ));

            /*****************  Huskylens test  *****************************/

            // 1. Read the blocks
            // 2. Print all blocks as String to telemetry (for debugging purposes)
            // 3. Identify the tag to align to (based on where the team prop is located)
            // 4. Strafe right or left to center the robot right in front of the targeted tag
            // 5. Turn left or right just the right amount to align robot with the backdrop using distance sensors

// TODO: Move the AprilTag read and strafe to a separate method
            blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            if (blocks.length == 0) {
                telemetry.update();
                continue;   // if no tags recognized, do not proceed; just restart the while loop and re-read blocks
            }

            // poll all block[i] and check if any of their id matches targetPos
            // targetBlock = i

            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());

                if(blocks[i].id == targetTagPos) targetBlockPos = i;
            }

            telemetry.addData("block of interest is in slot", targetBlockPos);
            telemetry.update();

            error = blocks[targetBlockPos].x - 160;
            telemetry.addData("error", error);

            // which way to strafe?
            if (error < 0)
                direction = -1;
            else {
                direction = 1;
            }

            // Strafe left or right to approach to the target tag
            if (!approached && Math.abs(error) > 10) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    0.0,
                                    0.2 * direction
                            ),
                            0.0
                    ));
            }
            else approached = true;

            drive.updatePoseEstimate();

            telemetry.addData("Approached=",approached);
            telemetry.addData("Aligned   =",aligned);

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            telemetry.update();

//                // check if aligned
//                distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.MM);
//                distanceRight = sensorDistanceRight.getDistance(DistanceUnit.MM);
//                if (Math.abs(distanceLeft - distanceRight) < 10) {
//                    aligned = true;
//                }
//            }


            //TODO: else here? what would i say?

        }
    }


    enum alliance{
        RED,
        BLUE
    }


    // Returns the position of the prop.
    // If not recognized, returns CENTER (2 or 5 depending on alliance)
    int getTargetTag(HuskyLens.Block[] blocks, alliance a) {

        int propPos;
        // for test purposes, return a known value
        // delete this segment when team prop is available
//        return 1;

        if (blocks.length == 1) {
            if (blocks[0].x < 120) {
                // Prop is on left
                propPos = (a==alliance.BLUE) ? 1 : 4;
            }
            else if (blocks[0].x > 200) {
                // prop is on right
                propPos = (a==alliance.BLUE) ? 3 : 6;
            }
            else {
                // prop is on center
                propPos = (a==alliance.BLUE) ? 2 : 5;
            }
        }
        else {
            // could not recognize; return center
            propPos = (a==alliance.BLUE) ? 2 : 5;
        }

        return propPos;
    }

    // Return turn angle in radian that is necessary to align the robot with the backdrop using distance sensors
    double getAlignmentAngle(DistanceSensor leftSensor, DistanceSensor rightSensor, Telemetry t) {

        double opposite, adjacent, hypotenuse;
        double left, right; // readings from sensors
        double turnAngle; // in radians

        int    turnDirection = 1;
        double epsilon = 0.035; // close enough (2 degrees) to return 0 in radians

        // Read distance sensors and calculate legs of the triangle
        left  = leftSensor.getDistance(DistanceUnit.MM);
        right = rightSensor.getDistance(DistanceUnit.MM);

        opposite   = Math.abs(left-right);
        adjacent   = DISTANCE_BETWEEN_DISTSENSORS * IN_TO_MM;
        hypotenuse = Math.sqrt(opposite*opposite + adjacent*adjacent);

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
        }
        else {
            turnDirection = (left>right) ? 1 : -1;
            turnAngle = turnDirection * Math.asin(opposite/hypotenuse);

            // don't bother to turn if the calculated angle is too small
            if(turnAngle<epsilon) turnAngle = 0.00;
        }

        t.addData("turnAngle = ", Math.toDegrees(turnAngle));
        t.update();

        return turnAngle;
    }
}
