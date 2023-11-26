package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoAbstractOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@Config
@Autonomous(name="Robot: Red-Left Auto", group="Robot")
public class AutoRL extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_RedLeft;
    }

    @Override
    public Action traj_left(MecanumDrive drive, BrainSTEMRobot robot) {
        return drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .setReversed(true)
                .splineTo(constants.vRedLeftSpike_Left, Math.toRadians(90))
                .lineToY(constants.vRedLeftSpike_Left.y + constants.robot_length / 2)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vRedClearStageGate, Math.toRadians(0))     // First clear the trusses
                .splineTo(constants.vRedBackdrop_Left, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_center(MecanumDrive drive, BrainSTEMRobot robot) {
        return drive.actionBuilder(startPose())
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vRedRightSpike_Center.y + constants.robot_length / 4)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vRedBackdrop_Center, Math.toRadians(0))     // Then, go to designated tag position

                .build();
    }

    @Override
    public Action traj_right(MecanumDrive drive, BrainSTEMRobot robot) {
        return drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vRedLeftSpike_Right.y, Math.toRadians(0))
                .endTrajectory()
                .lineToX(constants.vRedLeftSpike_Right.x - constants.robot_length / 2)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER, -constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(0))
                .splineTo(constants.vRedClearStageGate, Math.toRadians(0))
                .splineTo(constants.vRedBackdrop_Right, Math.toRadians(0))
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.RED;
    }
}