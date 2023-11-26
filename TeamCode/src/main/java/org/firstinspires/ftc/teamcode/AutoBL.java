package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

public class AutoBL extends AutoAbstractOpMode{

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_BlueLeft;
    }

    @Override
    public Action traj_left(MecanumDrive drive, BrainSTEMRobot robot) {
        return drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                .splineTo(constants.vBlueLeftSpike_Left, Math.toRadians(90))
                .lineToY(constants.vBlueLeftSpike_Left.y + constants.robot_length / 2)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vBlueClearStageGate, Math.toRadians(0))     // First clear the trusses
                .splineTo(constants.vBlueBackdrop_Left, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_center(MecanumDrive drive, BrainSTEMRobot robot) {
        return drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vBlueLeftSpike_Center.y + constants.robot_length / 4)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vBlueBackdrop_Center, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }
    @Override
    public Action traj_right(MecanumDrive drive, BrainSTEMRobot robot) {
        return drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vBlueLeftSpike_Right.y, Math.toRadians(0))
                .endTrajectory()
                .lineToX(constants.vBlueLeftSpike_Right.x - constants.robot_length / 2)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .splineTo(constants.vBlueBackdrop_Right, Math.toRadians(0))
                .splineTo(constants.vBlueBackdrop_Center, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Alliance alliance() {
        return null;
    }
}