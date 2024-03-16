package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        return robot.drive.actionBuilder(robot.drive.pose)
                .setReversed(true)

//                .splineToLinearHeading(new Pose2d(28, 41, Math.toRadians(0)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(23, 26), Math.toRadians(90))
                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
//                .lineToYSplineHeading(constants.vBlueLeftSpike_Left.y, Math.toRadians(0))
//                .endTrajectory()
////                        .lineToX(vBlueLeftSpike_Left.x - robot_length / 2)
//
//                // Drop yellow pixel in position
                .stopAndAdd(robot.drawbridge.drawBridgeUp)
//
//                // Discontinue trajectory
//                .endTrajectory()
//                .setReversed(true)
//
//                // Goto Backdrop to place your purple pixel
//                .setTangent(90)
//                .splineToSplineHeading(new Pose2d(constants.vBlueBackdrop_Left.x,constants.vBlueBackdrop_Left.y,Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position
                .splineToLinearHeading(new Pose2d(30, 45, Math.toRadians(180)), Math.toRadians(60))

                .stopAndAdd(robot.intake.intakeSafeAutos)
                .stopAndAdd(robot.lift.liftToGroundStateSafeAuto)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.depositor.bothDepositorPickupSafe)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(41, 45))
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                // Replace prop with your yellow pixel (just push)
//                .lineToY(constants.vBlueLeftSpike_Center.y - constants.robot_length/2.0 + 2.0)
                .lineToY(robot.drive.pose.position.y - 6.0)
                .stopAndAdd(robot.drawbridge.drawBridgeUp)

//                .endTrajectory()
                .setReversed(true)  // re-set reverse after .stopAndAdd as it loses config

                .splineToLinearHeading(new Pose2d(30, 41, Math.toRadians(180)), Math.toRadians(60))

                .stopAndAdd(robot.intake.intakeSafeAutos)
                .stopAndAdd(robot.lift.liftToGroundStateSafeAuto)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.depositor.bothDepositorPickupSafe)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(41, 41))
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//
//                // Go to backdrop to place your purple pixel
//                .setTangent(-45)
//                .lineToY(constants.vBlueLeftSpike_Center.y - 12)    // Move away from the pixel to avoid de-scoring
//                .setTangent(0)
//                .splineToSplineHeading(new Pose2d(constants.vBlueBackdrop_Center.x - 5.0, constants.vBlueBackdrop_Center.y, Math.toRadians(180)), Math.toRadians(0))     // Then, go to designated tag position
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                .splineToLinearHeading(new Pose2d(-3, 36, Math.toRadians(180)), Math.toRadians(170))


                // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
//                .lineToYSplineHeading(constants.vBlueLeftSpike_Right.y, Math.toRadians(180)) //+ 3.0, Math.toRadians(180))
//                .endTrajectory()
//                .lineToX(constants.vBlueLeftSpike_Right.x + constants.robot_length / 2.0 - 1.5)
//
//                // Drop yellow pixel in position
                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                .setReversed(true)

                .splineToConstantHeading(new Vector2d(30, 33), Math.toRadians(0))

                .stopAndAdd(robot.intake.intakeSafeAutos)
                .stopAndAdd(robot.lift.liftToGroundStateSafeAuto)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.depositor.bothDepositorPickupSafe)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(41, 33))
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//
//                // Discontinue trajectory
//                .endTrajectory()
//                .setReversed(true)
//
//                // Goto Backdrop to place your purple pixel
//                .splineTo(constants.vBlueBackdrop_Right, Math.toRadians(0))     // Then, go to designated tag position
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
    public Action cycle(BrainSTEMRobotA robot) {
        return null;
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(180)
//                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X, 75, Math.toRadians(180)), Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(robot.drive.pose.position.x, 62),Math.toRadians(-90))
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
