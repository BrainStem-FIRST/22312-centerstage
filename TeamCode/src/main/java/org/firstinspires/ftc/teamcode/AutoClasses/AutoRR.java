package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Robot: Red-Right Auto", group="Robot")
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

                .lineToYSplineHeading(constants.vRedRightSpike_Left.y - 3.0, Math.toRadians(180))
                .endTrajectory()
                .lineToX(constants.vRedRightSpike_Left.x + constants.robot_length / 2.0 - 1.5)    // Adjust delta accordingly

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
                .splineTo(constants.vRedBackdrop_Left, Math.toRadians(0))

                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedRight)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vRedRightSpike_Center.y + constants.robot_length/2.0 + 3.0)  // Adjust delta to fine tune pixel drop off position

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .setTangent(45)
                .lineToY(constants.vRedRightSpike_Center.y + 12)    // Move away from the pixel to avoid de-scoring
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Center.x, constants.vRedBackdrop_Right.y, Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position

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

                // Add small movement if needed
//                .lineToX(constants.vRedRightSpike_Right.x - constants.robot_length / 2)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .setTangent(45)
                .lineToX(constants.vRedRightSpike_Right.x-15)   // Move away from the pixel. Adjust distance if needed.
                .setTangent(225)
                .splineToSplineHeading(new Pose2d(constants.vRedBackdrop_Right.x,constants.vRedBackdrop_Right.y,Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position

                .build();
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(37)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X, -56, Math.toRadians(180)), Math.toRadians(-45))
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    public Orientation orientation() {
        return Orientation.RIGHT;
    }
}