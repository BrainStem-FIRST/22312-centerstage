package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Robot: Blue-Right Auto", group="Robot")
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

                // Go to position to drop purple pixel (this is a little next to the team prop, not pushing it)
                .setTangent(180)
                .lineToYLinearHeading(constants.vBlueRightSpike_Left.y + 5.0, Math.toRadians(0)) // -3.0
                .endTrajectory()
                .lineToX(constants.vBlueRightSpike_Left.x - constants.robot_length / 2.0)
                .endTrajectory()
// probably not needed
//                .lineToX(constants.vBlueRightSpike_Left.x - 5.0 - constants.robot_length / 2) // no delta

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER, constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.0000)), Math.toRadians(0))
                .splineTo(new Vector2d(constants.vBlueClearStageGate.x + 5.0, constants.vBlueClearStageGate.y), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineTo(constants.vBlueBackdrop_Left, Math.toRadians(0))

                .build();

    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vBlueRightSpike_Center.y - constants.robot_length / 2.0 + 4.0)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
//                .lineToY(constants.vBlueRightSpike_Center.y-12)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x + 5.0, constants.vBlueClearStageGate.y - 3.0, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel

//                .setTangent(Math.toRadians(-45))
//                .splineTo(new Vector2d(constants.vBlueClearStageGate.x + 5.0, constants.vBlueClearStageGate.y), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineTo(new Vector2d(constants.vBlueBackdrop_Center.x + 3.0 , constants.vBlueBackdrop_Center.y), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go backwards
                .setReversed(true)

                .splineTo(constants.vBlueRightSpike_Right, Math.toRadians(-90))
                .lineToY(constants.vBlueRightSpike_Right.y - constants.robot_length / 2.0 + 2.0)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
                .splineTo(new Vector2d(constants.vBlueClearStageGate.x + 5.0, constants.vBlueClearStageGate.y - 3.0), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
//                .splineTo(constants.vBlueClearStageGate, Math.toRadians(0))
                .splineTo(new Vector2d(constants.vBlueBackdrop_Right.x + 1.5, constants.vBlueBackdrop_Right.y), Math.toRadians(0))     // Then, go to designated tag position
                .build();

    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(37)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X, 12, Math.toRadians(180)), Math.toRadians(-45))
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    public Orientation orientation() {
        return Orientation.RIGHT;
    }
}