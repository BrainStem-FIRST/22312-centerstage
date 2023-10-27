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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous (name="Robot: Auto Test", group="Robot")
public class AutoTest extends ActionOpMode {

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

        HuskyLens huskyLens;
        int direction = 1;
        int error;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        HuskyLens.Block[] blocks;

        DistanceSensor sensorDistanceLeft, sensorDistanceRight;
        sensorDistanceLeft = hardwareMap.get(DistanceSensor.class, "sensor_distanceLeft");
        DistanceSensor sensorDistance2;
        sensorDistanceRight = hardwareMap.get(DistanceSensor.class, "sensor_distanceRight");

        double distanceLeft = 0;
        double distanceRight = 0;
        boolean approached = false; // indicator that the robot is close enough to the desired tag

        waitForStart();

        while (opModeIsActive()) {

//            runBlocking(trajectory);

            /*****************  Huskylens test  *****************************/

            // 1. Read the blocks
            // 2. Print all blocks as String to telemetry (for debugging purposes)
            // 3. Identify the tag at the center
            //    a) Do you see 3 tags? The one in the middle (compare x values) is the Center one.
            //    b) Do you see 2 tags? Where are they relative to the robot? If the tags are towards
            //       the right side of the screen, robot is on the left. The center tag is the rightmost tag.
            //       If the tags are towards the left, the robot is on the right. The center tag is the leftmost
            //       tag.
            //    c) Do you see only one tag? Cannot determine if it is the center tag unless we know the exact ID.
            //       Experiment by looking at telemetry if the center tag always returns the same ID.
            // 4. Strafe right or left to center the robot right in front of the center tag.

            blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);

            if (blocks.length == 0) {
                telemetry.update();
                continue;
            }

            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }

            telemetry.addData("center id", center(blocks));

            error = blocks[center(blocks)].x-160;
            telemetry.addData("error", error);

            if (error < 0)
                direction = -1;
            else {
                direction = 1;
            }

            if (!approached && Math.abs(error) > 2) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                0.0,
                                0.2*direction
                        ),
                        0.0
                ));
            }
            else {
                approached = true;
            }

            drive.updatePoseEstimate();

            telemetry.addData("Approached=",approached);
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            telemetry.update();

            distanceLeft = sensorDistanceLeft.getDistance(DistanceUnit.CM);
            distanceRight = sensorDistanceRight.getDistance(DistanceUnit.CM);

            telemetry.addData("Left distance =",distanceLeft);
            telemetry.addData("Right distance=",distanceRight);
            telemetry.update();

            if(approached) {

                if(Math.abs(distanceRight-distanceLeft)>1) {
                    if (distanceLeft < distanceRight) {
                        double turnAngle = Math.asin(distanceRight - distanceLeft / 14);
                        telemetry.addData("Turn Angle = ", Math.toRadians(turnAngle));

                        runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .turn(turnAngle) //TODO: what is robot's horizontal length?? also which direction is this??
                                        .build()
                        );
                    } else {
                        double turnAngle = Math.asin(distanceLeft - distanceRight / 14);
                        telemetry.addData("Turn Angle = ", Math.toRadians(turnAngle));

                        runBlocking(
                                drive.actionBuilder(drive.pose)
                                        .turn(Math.asin(distanceLeft - distanceRight / 14)) //TODO: what is robot's horizontal length?? also which direction is this??
                                        .build()
                        );
                    }
                }
            }

            telemetry.update();

            //TODO: else here? what would i say?

        }
    }

    int center(HuskyLens.Block[] b) {
        int centerId = -1;

        if (b.length == 3) {
            if (b[0].x < b[1].x) { //if 0 is left of 1
                if (b[1].x < b[2].x) {
                    centerId = 1;
                } else if (b[2].x < b[0].x) {
                    centerId = 0;
                } else {
                    centerId = 2;
                }
            } else if (b[1].x < b[2].x) {
                if (b[2].x < b[0].x) {
                    centerId = 2;
                } else {
                    centerId = 0;
                }
            } else {
                centerId = 1;
            }
        }
        else if (b.length == 2) {
            // first find the relative order
            int left, right;
            if (b[0].x < b[1].x){
                left = 0;
                right = 1;
            }
            else {
                left = 1;
                right = 0;
            }

            // then compare and find whichever is closer to the edge
            if (b[left].x < 320-b[right].x) {
                centerId = left;
            }
            else {
                centerId = right;
            }
        }
        else if (b.length == 1) {
            centerId = 0;
        }

        return centerId;
    }



}