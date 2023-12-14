package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class AutoBL extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_BlueLeft;
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueLeft)
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vBlueLeftSpike_Left.y, Math.toRadians(0))
                .endTrajectory()
//                        .lineToX(vBlueLeftSpike_Left.x - robot_length / 2)

                // Drop yellow pixel in position

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(constants.vBlueBackdrop_Left.x,constants.vBlueBackdrop_Left.y,Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vBlueLeftSpike_Center.y - 13.0 + constants.robot_length / 4)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .splineTo(new Vector2d(constants.vBlueBackdrop_Center.x, constants.vBlueBackdrop_Center.y + 4.0), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }
    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vBlueLeftSpike_Right.y, Math.toRadians(180))
                .endTrajectory()
//                        .lineToX(vBlueLeftSpike_Left.x - robot_length / 2)

                // Drop yellow pixel in position

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(constants.vBlueBackdrop_Right.x,constants.vBlueBackdrop_Right.y,Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(startPose())
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(50, 12), Math.toRadians(0))
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.BLUE;
    }
}
