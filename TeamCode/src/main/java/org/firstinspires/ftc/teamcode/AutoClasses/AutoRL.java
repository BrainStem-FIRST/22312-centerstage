package org.firstinspires.ftc.teamcode.AutoClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Mat;

@Config
@Autonomous(name="Robot: Red-Left Auto", group="Robot")
public class AutoRL extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_RedLeft;
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                .setTangent(-135)
                .lineToX(constants.vRedLeftSpike_Left.x)
//                .splineTo(new Vector2d(constants.vRedLeftSpike_Left.x, constants.vRedLeftSpike_Left.y - 4.0), Math.toRadians(90))
//                .lineToY(constants.vRedLeftSpike_Left.y + constants.robot_length / 2.0)

                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, (-constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180.00001)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedClearStageGate.x-8, constants.vRedClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Left.x, constants.vRedBackdrop_Left.y + 12, Math.toRadians(-180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action traj_init(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("start pose:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })
                .setReversed(true)

                // Move close enough to center spike
                .lineToY(-35)

                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose before findSpike:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })

                .stopAndAdd(findSpike(robot))

                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose after traj_init:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })

                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                // move to position to drop purple pixel - relative to where robot stopped after seeing the center spike
                .lineToY(robot.drive.pose.position.y + 6.0)
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose before spit:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })

                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, (-constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180)), Math.toRadians(135))
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose after spline:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedClearStageGate.x-8, constants.vRedClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Center.x, constants.vRedBackdrop_Center.y + 12, Math.toRadians(-180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose before traj_right:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
//                    telemetry.update();
                    return false;
                })

                // go forwards
                .setReversed(false)

                // Turn heading towards right spike
                .setTangent(Math.toRadians(-35))
                .lineToYLinearHeading(-36, Math.toRadians(-25))
                .lineToX(-24)
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose after before spit:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })
                // Drop yellow pixel in position
                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, (-constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180.00001)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedClearStageGate.x-8, constants.vRedClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Right.x-4, constants.vRedBackdrop_Right.y + 12, Math.toRadians(-180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action cycle(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, (-constants.TILE_CENTER_TO_CENTER / 2) + 12, Math.toRadians(-180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER * 2.5, (-constants.TILE_CENTER_TO_CENTER / 2) + 12, Math.toRadians(-180)), Math.toRadians(-180))
                .waitSeconds(2.0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, (-constants.TILE_CENTER_TO_CENTER / 2) + 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Left.x+2, (constants.vRedBackdrop_Left.y-2) + 12, Math.toRadians(180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(37)  /// Add if needed
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X,-20 + 12, Math.toRadians(180)), Math.toRadians(45))
                .build();
    }


    @Override
    public Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    public Orientation orientation() {
        return Orientation.LEFT;
    }
}
