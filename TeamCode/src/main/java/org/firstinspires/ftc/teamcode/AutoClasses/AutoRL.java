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

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
//                .splineTo(constants.vRedClearStageGate, Math.toRadians(0))     // First clear the trusses
//                .splineTo(new Vector2d(constants.vRedBackdrop_Left.x - 6.5, constants.vRedBackdrop_Left.y - 1.5), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_init(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                // Move close enough to center spike
                .lineToY(-35)

                .stopAndAdd(telemetryPacket -> {
                    telemetry.addData("pose before findSpike", robot.drive.pose.position.y);
                    telemetry.update();
                    return false;
                })

                .stopAndAdd(findSpike(robot))

                .stopAndAdd(telemetryPacket -> {
                    telemetry.addData("pose after findSpike", robot.drive.pose.position.y);
                    telemetry.update();
                    return false;
                })

                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setReversed(true)

                // move to position to drop purple pixel - relative to where robot stopped after seeing the center spike
                .lineToY(robot.drive.pose.position.y + 8.0)

                .stopAndAdd(telemetryPacket -> {
                    telemetry.addData("pose target", robot.drive.pose.position.y);
                    telemetry.update();
                    return false;
                })

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()

//                .setReversed(true)
//                .splineTo(new Vector2d(constants.vRedClearStageGate.x, constants.vRedClearStageGate.y - 2.0), Math.toRadians(0))
//                .splineTo(new Vector2d(constants.vRedBackdrop_Center.x - 8.0, constants.vRedBackdrop_Center.y - 1.5), Math.toRadians(0))     // Then, go to designated tag position

                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose after finding Spike:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.update();
                    return false;
                })

                // go forwards
                .setReversed(true)
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(constants.vRedLeftSpike_Right.x-8, -30,Math.toRadians(-35)),Math.toRadians(0))

                .setTangent(-90)
//                .splineToLinearHeading(new Pose2d(-35, -30,Math.toRadians(-35)),Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(robot.drive.pose.position.x + 5.0,
//                        robot.drive.pose.position.y - 5.0, Math.toRadians(-35)), Math.toRadians(0))

                .splineTo(new Vector2d(robot.drive.pose.position.x + 5.0,
                        robot.drive.pose.position.y - 5.0), Math.toRadians(-35))

//                .lineToY(-30)

//                .stopAndAdd(telemetryPacket -> {
//                    telemetry.addLine("Pose after LineToY:");
//                    telemetry.addData("x", robot.drive.pose.position.x);
//                    telemetry.addData("y", robot.drive.pose.position.y);
//                    telemetry.update();
//                    return false;
//                })
                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Goto Backdrop to place your yellow pixel
//                .setTangent(Math.toRadians(135))
//                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER, -constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(0))
//                .splineTo(new Vector2d(constants.vRedClearStageGate.x + 5.0, constants.vRedClearStageGate.y), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
//                .splineTo(new Vector2d(constants.vRedBackdrop_Right.x - 6.0, constants.vRedBackdrop_Right.y - 3.0), Math.toRadians(0))
                .build();
    }


    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(37)  /// Add if needed
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X,-20, Math.toRadians(180)), Math.toRadians(45))
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
