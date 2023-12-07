package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class AutoRR extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_RedRight;
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedRight)
                // go backwards
                .setReversed(true)

                .splineTo(constants.vRedRightSpike_Left, Math.toRadians(90))
                .lineToY(constants.vRedRightSpike_Left.y + constants.robot_length / 2)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vRedClearStageGate, Math.toRadians(0))     // First clear the trusses
                .splineTo(constants.vRedBackdrop_Left, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedRight)
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
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedRight)
                // go backwards
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vRedRightSpike_Right.y, Math.toRadians(0))
                .endTrajectory()
                .lineToX(constants.vRedRightSpike_Right.x - constants.robot_length / 2)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .splineTo(constants.vRedBackdrop_Right, Math.toRadians(0))
                .splineTo(constants.vRedBackdrop_Center, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(startPose())
                .splineTo(new Vector2d(46, -12), Math.toRadians(-60))
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.RED;
    }
}