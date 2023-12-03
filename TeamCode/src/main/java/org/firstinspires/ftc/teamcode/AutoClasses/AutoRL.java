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
    public Action traj_left(MecanumDrive drive, BrainSTEMRobotA robot) {
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
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(startPose())
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
                .lineToY(constants.vRedLeftSpike_Center.y + constants.robot_length / 4)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

//                .setTangent(0)
                // Go to backdrop to place your purple pixel
                .splineTo(constants.vRedClearStageGate, Math.toRadians(0))
                .splineToConstantHeading(constants.vRedBackdrop_Center, Math.toRadians(0))
                //i set it to right not center thats why wasnt workigngggg adlksjfa;lkdjsa;lkdsf
//                .stopAndAdd(robot.lift.raiseLiftAuto)

                .build();
    }

    @Override
    public Action traj_right(MecanumDrive drive, BrainSTEMRobotA robot) {
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
                .build();
    }

    @Override
    public Alliance alliance() {
        return Alliance.RED;
    }
}
