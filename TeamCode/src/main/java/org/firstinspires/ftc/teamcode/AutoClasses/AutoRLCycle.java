package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Red-Left Auto Cycle", group="Robot")
public class AutoRLCycle extends AutoAbstractOpModeCycle {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_RedLeft;
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                .setTangent(-135)
                .lineToX(constants.vRedLeftSpike_Left.x)

                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                .endTrajectory()
                .setReversed(true)

                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, (-constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180.00001)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedClearStageGate.x-8, constants.vRedClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Left.x, constants.vRedBackdrop_Left.y + 12, Math.toRadians(-180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action traj_init(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("start pose:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })
                .setReversed(true)

                // Move close enough to center spike
                .lineToY(-35)

                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose before findSpike:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })

                .stopAndAdd(findSpike(robot))
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose after traj_init:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })

                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .setReversed(true)
                .afterDisp(40, robot.drawbridge.drawBridgeUp)
                .splineToLinearHeading(new Pose2d(-32, -12, Math.toRadians(180)), Math.toRadians(90))
                .afterDisp(55, robot.intake.intakePixel)
                .splineToConstantHeading(new Vector2d(-50, -4), Math.toRadians(180))
//                .waitSeconds(1.0)
                .afterDisp(0, robot.intake.intakePixel)
                .splineToConstantHeading(new Vector2d(-57, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-56, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .afterDisp(0, robot.intake.intakePixel)
                .splineToConstantHeading(new Vector2d(-56.25, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5, 5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(5, robot.lift.lowerLiftToGroundState)
                .afterDisp(40, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(51, -24), Math.toRadians(-45))
                .waitSeconds(0.5)
                .setReversed(false)
                .afterDisp(5, robot.wrist.turnWristNinety)
                .afterDisp(5, robot.arm.armToIdle)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .splineToConstantHeading(new Vector2d(0, -8), Math.toRadians(180))
                .afterDisp(0, robot.lift.lowerLiftToIdleState)
                .afterDisp(40, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-50, -4), Math.toRadians(180))
                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeight)
                .afterDisp(0, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-57.5, -4), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(10, robot.lift.lowerLiftToGroundState)
                .afterDisp(40, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftToMiddleState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(51.5, -24), Math.toRadians(-45))
//                // move to position to drop purple pixel - relative to where robot stopped after seeing the center spike
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose before traj_right:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
//                    telemetry.update();
                    return false;
                })

                // go forwards
                .setReversed(false)

                // Turn heading towards right spike
                .setTangent(Math.toRadians(-35))
                .lineToYLinearHeading(-36, Math.toRadians(-25))
                .lineToX(-24)
                .stopAndAdd(telemetryPacket -> {
                    telemetry.addLine("Pose after before spit:");
                    telemetry.addData("x", robot.drive.pose.position.x);
                    telemetry.addData("y", robot.drive.pose.position.y);
                    telemetry.addData("heading", Math.toDegrees(robot.drive.pose.heading.log()));
                    telemetry.update();
                    return false;
                })
                // Drop yellow pixel in position
                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, (-constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180.00001)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vRedClearStageGate.x-8, constants.vRedClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Right.x-4, constants.vRedBackdrop_Right.y + 12, Math.toRadians(-180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action cycle(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(90)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, (-constants.TILE_CENTER_TO_CENTER / 2) + 12, Math.toRadians(-180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER * 2.5, (-constants.TILE_CENTER_TO_CENTER / 2) + 12, Math.toRadians(-180)), Math.toRadians(-180))
                .waitSeconds(2.0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, (-constants.TILE_CENTER_TO_CENTER / 2) + 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(constants.vRedBackdrop_Left.x+2, (constants.vRedBackdrop_Left.y-2) + 12, Math.toRadians(180)), Math.toRadians(-90))
                .build();
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
//                .lineToX(37)  /// Add if needed
                .setTangent(135)
                .splineToLinearHeading(new Pose2d(constants.FIELD_BACKSTAGE_X,-20 + 12, Math.toRadians(180)), Math.toRadians(45))
                .build();
    }

//    @Override
//    public Action deposit_right(@NonNull BrainSTEMRobotA robot){
//        return robot.drive.actionBuilder(robot.drive.pose)
//                .setReversed(true)
//                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
//                .stopAndAdd(robot.arm.armToDeposit)
//                .strafeToConstantHeading(new Vector2d(52, -30))
//                .stopAndAdd(robot.wrist.turnWristOneEighty)
//                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//                .build();
//    }
//
//    public Action deposit_center(BrainSTEMRobotA robot){
//        return robot.drive.actionBuilder(robot.drive.pose)
//                .setReversed(true)
//                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
//                .stopAndAdd(robot.arm.armToDeposit)
//                .strafeToConstantHeading(new Vector2d(52, -24))
//                .stopAndAdd(robot.wrist.turnWristOneEighty)
//                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//                .build();
//    }
//
//    public Action deposit_left(BrainSTEMRobotA robot){
//        return robot.drive.actionBuilder(robot.drive.pose)
//                .setReversed(true)
//                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
//                .stopAndAdd(robot.arm.armToDeposit)
//                .strafeToConstantHeading(new Vector2d(52, -18))
//                .stopAndAdd(robot.wrist.turnWristOneEighty)
//                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//                .build();
//
//    }

    @Override
    public Alliance alliance() {
        return Alliance.RED;
    }

    @Override
    public Orientation orientation() {
        return Orientation.LEFT;
    }
}
