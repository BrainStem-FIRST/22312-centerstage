package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Robot: Red-Left Auto", group="Robot")
public class AutoRL extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return new Pose2d(-1.5 * constants.TILE_CENTER_TO_CENTER, -constants.FIELD_BOUNDARY_FROM_CENTER + constants.robot_length / 2, Math.toRadians(-90));
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .setReversed(true)

                .splineTo(constants.vRedLeftSpike_Left, Math.toRadians(90))
                .lineToY(constants.vRedLeftSpike_Left.y + constants.robot_length / 2.0)

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
        return robot.drive.actionBuilder(startPose())
                // go backwards
                .setReversed(true)

                // Replace prop with your purple pixel (the offset is to adjust pixel's landing position after spit)
                .lineToY(constants.vRedLeftSpike_Center.y + constants.robot_length/2.0 - 3.0)

                .stopAndAdd(robot.intake.spitPixel)
                .endTrajectory()
                .setReversed(true)

                .setTangent(-45)
                .lineToY(constants.vBlueLeftSpike_Center.y - 12)    // Move away from the pixel to avoid de-scoring
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(constants.vBlueBackdrop_Center.x, constants.vBlueBackdrop_Center.y, Math.toRadians(180.0001)), Math.toRadians(0))     // Then, go to designated tag position

                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .setReversed(true)

                // Go to position to drop purple pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vRedLeftSpike_Right.y - 3.0, Math.toRadians(0))
                .endTrajectory()
// probably not needed
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
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(37)  /// Add if needed
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X,-12, Math.toRadians(180)), Math.toRadians(45))
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
