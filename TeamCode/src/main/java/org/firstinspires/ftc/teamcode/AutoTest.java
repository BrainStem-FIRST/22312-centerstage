package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

    double TILE_INSIDE_TO_INSIDE = 22.875;
    double TILE_CENTER_TO_CENTER = 23.625;
    double TILE_CENTER_TO_EDGE   = 23.25;
    double TILE_TEETH            = 0.75;

    double FIELD_BOUNDARY_FROM_CENTER = 2.0 * TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // White Pixel Stack positions
    Vector2d vRedStack_Inner   = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER+1.5,-FIELD_BOUNDARY_FROM_CENTER+35.0);
    Vector2d vRedStack_Center  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER+1.5,-FIELD_BOUNDARY_FROM_CENTER+46.875);
    Vector2d vRedStack_Outer   = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER+1.5,-FIELD_BOUNDARY_FROM_CENTER+58.75);

    Vector2d vBlueStack_Inner  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER+1.5,FIELD_BOUNDARY_FROM_CENTER-35.0);
    Vector2d vBlueStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER+1.5,FIELD_BOUNDARY_FROM_CENTER-46.875);
    Vector2d vBlueStack_Outer  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER+1.5,FIELD_BOUNDARY_FROM_CENTER-58.75);

    // Spike positions
    Vector2d vRedLeftSpike_Left   = new Vector2d(-TILE_CENTER_TO_CENTER-TILE_CENTER_TO_EDGE+0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-6.0);
    Vector2d vRedLeftSpike_Center = new Vector2d(-1.5*TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-0.5);
    Vector2d vRedLeftSpike_Right  = new Vector2d(-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-6.0);

    Vector2d vRedRightSpike_Left   = new Vector2d(TILE_TEETH/2.0+0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-6.0);
    Vector2d vRedRightSpike_Center = new Vector2d(0.5*TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-0.5);
    Vector2d vRedRightSpike_Right  = new Vector2d(TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-6.0);

    Vector2d vBlueRightSpike_Left   = new Vector2d(-TILE_CENTER_TO_CENTER-TILE_CENTER_TO_EDGE+0.5,TILE_CENTER_TO_CENTER+TILE_TEETH/2.0+6.0);
    Vector2d vBlueRightSpike_Center = new Vector2d(-1.5*TILE_CENTER_TO_CENTER,TILE_CENTER_TO_CENTER+TILE_TEETH/2.0+0.5);
    Vector2d vBlueRightSpike_Right  = new Vector2d(-TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-0.5,TILE_CENTER_TO_CENTER+TILE_TEETH/2.0+6.0);

    Vector2d vBlueLeftSpike_Left   = new Vector2d(TILE_TEETH/2.0+0.5,TILE_CENTER_TO_CENTER+TILE_TEETH/2.0+6.0);
    Vector2d vBlueLeftSpike_Center = new Vector2d(0.5*TILE_CENTER_TO_CENTER,TILE_CENTER_TO_CENTER+TILE_TEETH/2.0+0.5);
    Vector2d vBlueLeftSpike_Right  = new Vector2d(TILE_CENTER_TO_CENTER-TILE_TEETH/2.0-0.5,TILE_CENTER_TO_CENTER+TILE_TEETH/2.0+6.0);

    // Backstage positions
    double FIELD_BACKSTAGE_X = 2.0 * TILE_CENTER_TO_CENTER + TILE_TEETH/2.0 + 0.5;
    double FIELD_BACKDROP_X  = TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE + 13.5 - 2.0;    // 2 inches short of hitting the backdrop

    // Backdrop April Tag Positions
    Vector2d vRedBackdrop_Left   = new Vector2d(FIELD_BACKDROP_X,-1.5*TILE_CENTER_TO_CENTER+6);
    Vector2d vRedBackdrop_Center = new Vector2d(FIELD_BACKDROP_X,-1.5*TILE_CENTER_TO_CENTER);
    Vector2d vRedBackdrop_Right  = new Vector2d(FIELD_BACKDROP_X,-1.5*TILE_CENTER_TO_CENTER-6);

    Vector2d vBlueBackdrop_Left   = new Vector2d(FIELD_BACKDROP_X,1.5*TILE_CENTER_TO_CENTER-6);
    Vector2d vBlueBackdrop_Center = new Vector2d(FIELD_BACKDROP_X,1.5*TILE_CENTER_TO_CENTER);
    Vector2d vBlueBackdrop_Right  = new Vector2d(FIELD_BACKDROP_X,1.5*TILE_CENTER_TO_CENTER+6);

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        drive.pose = new Pose2d(-36, -61, Math.toRadians(90));
        Action trajectory =
                drive.actionBuilder(drive.pose)
                        .lineToY(-33)
                        .waitSeconds(1)

                        // Goto Backdrop to place your purple pixel
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(0)), Math.toRadians(0))
                        .lineToX(48)
                        .waitSeconds(1)

                        // Goto stack and collect 2 white pixels
                        .setReversed(true)
                        .setTangent(Math.toRadians(180))
                        .splineTo(new Vector2d(12, -12), Math.toRadians(180))
                        .lineToX(-61)
                        .waitSeconds(1)

                        // Goto Backstage and drop 2 white pixels
                        .setReversed(false)
                        .lineToX(12)
                        .setTangent(Math.toRadians(0))
                        .splineTo(new Vector2d(48, -36), Math.toRadians(0))
                        .waitSeconds(1)

                        .build();

        telemetry.addLine("Trajectory built");

        // Huskylens initialization (HW and Selection of algorithm)


        telemetry.update();

        int direction = 1;
        int error;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        HuskyLens.Block[] blocks;

        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "sensor_distanceLeft");
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "sensor_distanceRight");

        double distanceLeft = 0;
        double distanceRight = 0;
        boolean approached = false; // indicator that the robot is close enough to the desired tag
        boolean aligned = false;

        // TODO: Read blocks continuously until Start. Have some feedback to DS to confirm recognition

        // Recognize the team prop during init
        blocks = huskyLens.blocks();

        // Determine the prop position
        int targetTagPos = getTargetTag(blocks, alliance.BLUE);
        int targetBlockPos = 0; // The block of interest within the blocks array


        waitForStart();

        while (opModeIsActive()) {

//            runBlocking(trajectory);

            /*****************  Huskylens test  *****************************/

            // 1. Read the blocks
            // 2. Print all blocks as String to telemetry (for debugging purposes)
            // 3. Identify the tag to align to (based on where the team prop is located)
            // 4. Strafe right or left to center the robot right in front of the targeted tag
            // 5. Turn left or right just the right amount to align robot with the backdrop using distance sensors

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


            if(approached && !aligned) {

                runBlocking(
                        drive.actionBuilder(drive.pose)
                                .turn(getAlignmentAngle(sensorDistanceLeft, sensorDistanceRight, telemetry))
                                .build()
                );

                // check if aligned
                distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.MM);
                distanceRight = sensorDistanceRight.getDistance(DistanceUnit.MM);
                if (Math.abs(distanceLeft - distanceRight) < 10) {
                    aligned = true;
                }
            }

/************************** from wednesday  **********************************************************************

                distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.MM);
                distanceRight = sensorDistanceRight.getDistance(DistanceUnit.MM);
                opposite = Math.abs(distanceLeft - distanceRight);

                telemetry.addData("Left distance =",distanceLeft);
                telemetry.addData("Right distance=",distanceRight);
                telemetry.addData("Opposite =",opposite);

//
//                if (opposite > hypoteneuse){
//                    if (distanceLeft > distanceRight) {
//                        runBlocking(
//                                drive.actionBuilder(drive.pose)
//                                        .turn(Math.toRadians(-10))
//                                        .build()
//                        );
//                    }
//                    else if (distanceRight > distanceLeft) {
//                        runBlocking(
//                                drive.actionBuilder(drive.pose)
//                                        .turn(Math.toRadians(10))
//                                        .build()
//                        );
//                    }
//                }

                if(opposite > 10) {

                    turnAngle_rad = Math.asin(opposite / hypoteneuse);
                    telemetry.addData("Turn Angle = ", Math.toRadians(turnAngle_rad));

                    if (distanceLeft < distanceRight) {

                        //get inverse sin radians for how much the robot should turn

                        runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .turn(turnAngle_rad) //TODO: what is robot's horizontal length?? also which direction is this??
                                        .build()
                        );
                    } else {
                        runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .turn(-turnAngle_rad) //TODO: what is robot's horizontal length?? also which direction is this??
                                        .build()
                        );

                    }
                }
                else aligned = true;
            }

            telemetry.update();
 *************************************************************************************************************/

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
