package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name="Robot: Blue-Left Auto", group="Robot")
public class AutoBL extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_BlueLeft;
    }

    @Override
    public Action traj_init(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                // Move close enough to center spike
                .lineToY(35)

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
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueLeft)
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vBlueLeftSpike_Left.y, Math.toRadians(0))
                .endTrajectory()
//                        .lineToX(vBlueLeftSpike_Left.x - robot_length / 2)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

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
                .lineToY(constants.vBlueLeftSpike_Center.y - constants.robot_length/2.0 + 2.0)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .setTangent(-45)
                .lineToY(constants.vBlueLeftSpike_Center.y - 12)    // Move away from the pixel to avoid de-scoring
                .setTangent(0)
                .splineToSplineHeading(new Pose2d(constants.vBlueBackdrop_Center.x - 5.0, constants.vBlueBackdrop_Center.y, Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueLeft)
                // go backwards
                .setReversed(true)

                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                .lineToYSplineHeading(constants.vBlueLeftSpike_Right.y, Math.toRadians(180)) //+ 3.0, Math.toRadians(180))
                .endTrajectory()
                .lineToX(constants.vBlueLeftSpike_Right.x + constants.robot_length / 2.0 - 1.5)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .splineTo(constants.vBlueBackdrop_Right, Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action deposit_right(BrainSTEMRobotA robot) {
        return null;
    }

    @Override
    public Action deposit_center(BrainSTEMRobotA robot) {
        return null;
    }

    @Override
    public Action deposit_left(BrainSTEMRobotA robot) {
        return null;
    }

    @Override
    public Action cycle(BrainSTEMRobotA robot) {
        return null;
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X, 56, Math.toRadians(180)), Math.toRadians(45))
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    public Orientation orientation() {
        return Orientation.LEFT;
    }
}
