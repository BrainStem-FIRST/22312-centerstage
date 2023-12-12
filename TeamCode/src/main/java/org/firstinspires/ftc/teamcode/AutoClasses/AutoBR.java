package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class AutoBR extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_BlueRight;
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
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
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vBlueRightSpike_Center.y + constants.robot_length / 2)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vBlueClearStageGate, Math.toRadians(0))      // First clear the trusses
                .splineTo(constants.vBlueBackdrop_Center, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
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
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER, -constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(0))
                .splineTo(constants.vBlueClearStageGate, Math.toRadians(0))
                .splineTo(constants.vBlueBackdrop_Right, Math.toRadians(0))
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
        return Alliance.BLUE;
    }
}