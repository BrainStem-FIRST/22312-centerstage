package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Red-Right Auto", group="Robot")
public class AutoRR extends AutoAbstractOpMode {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_RedRight;
    }

    @Override
    public Action traj_init(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedRight)
                // go backwards
                .setReversed(true)

                // Move close enough to center spike
                .lineToY(-28)

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
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

//                .lineToYSplineHeading(constants.vRedRightSpike_Left.y, Math.toRadians(180))
//                .endTrajectory()
                .lineToX(constants.vRedRightSpike_Left.x + constants.robot_length / 2.0 + 1.5)    // Adjust delta accordingly

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)

                // Go to backdrop to place your purple pixel
                .splineTo(new Vector2d(constants.vRedBackdrop_Left.x - 4, constants.vRedBackdrop_Left.y), Math.toRadians(0))

                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)
                // Replace prop with your yellow pixel (just push)
                .lineToY(robot.drive.pose.position.y + 6.0)
                .stopAndAdd(robot.drawbridge.drawBridgeUp)
                .afterDisp(5, robot.lift.raiseLiftAutoToLowState)

                // re-set reverse after .stopAndAdd as it loses config

                // Go to backdrop to place your purple pixel
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Center.x - 4.0, constants.vRedBackdrop_Center.y + 9, Math.toRadians(180)), Math.toRadians(0))
                .setReversed(true)
//                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(54, -27))
                .stopAndAdd(robot.wrist.turnWristOneEighty)
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)// Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)
/*
                .splineTo(new Vector2d(constants.vRedRightSpike_Right.x, constants.vRedRightSpike_Right.y - 5.0), Math.toRadians(0))
//                        .lineToYSplineHeading(vBlueLeftSpike_Left.y, Math.toRadians(0))
                .endTrajectory()
 */
                .lineToX(constants.vRedRightSpike_Right.x + constants.robot_length / 2+ 12)

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Discontinue trajectory
                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your purple pixel
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(constants.vRedBackdrop_Right.x-4, constants.vRedBackdrop_Right.y), Math.toRadians(0))     // Then, go to designated tag position

                .build();
    }

    @Override
    public Action cycle(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, -constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(-180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER * 2.5, -constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(-180)), Math.toRadians(-180))
                .waitSeconds(2.0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, -constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Left.x+2, constants.vRedBackdrop_Left.y-2, Math.toRadians(180)), Math.toRadians(-90))
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

    public Action deposit_right(BrainSTEMRobotA robot){
        return robot.drive.actionBuilder(robot.drive.pose)
                .setReversed(true)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(52, -30))
                .stopAndAdd(robot.wrist.turnWristOneEighty)
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
                .build();
    }

    public Action deposit_center(BrainSTEMRobotA robot){
        return robot.drive.actionBuilder(robot.drive.pose)
                .setReversed(true)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(52, -24))
                .stopAndAdd(robot.wrist.turnWristOneEighty)
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
                .build();
    }

    public Action deposit_left(BrainSTEMRobotA robot){
        return robot.drive.actionBuilder(robot.drive.pose)
                .setReversed(true)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(52, -18))
                .stopAndAdd(robot.wrist.turnWristOneEighty)
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
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