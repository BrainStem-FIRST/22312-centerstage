package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;

@Config
@Autonomous(name="Robot: Red-Left Auto Cycle Intake Sensors", group="Robot")
public class AutoRLCycleWithIntakeSensors extends AutoAbstractOpModeCycle {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_RedLeft;
    }

    @Override
    public Action traj_left(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                // go backwards
                .setReversed(true)
                .afterDisp(32, robot.drawbridge.drawBridgeAllTheWayUp)
                .afterDisp(50, robot.drawbridge.drawBridgeUp)
                .splineToLinearHeading(new Pose2d(-40, -6, Math.toRadians(180)), Math.toRadians(90))
//                .afterDisp(65, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-50, -5), Math.toRadians(180))
//                .waitSeconds(1.0)
                .afterDisp(0, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-57.25, -5), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-56, -5), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
//                .afterDisp(0, robot.intake.intakePixelLeftPixelSpike)
                .splineToConstantHeading(new Vector2d(-57.25, -5), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5, 5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(15,robot.intake.intakeSafeAutos)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .afterDisp(60, robot.depositor.bothDepositorsPickup)
//                .afterDisp(60, robot.intake.spitPixel)
//                .afterDisp(80, robot.lift.raiseLiftAutoToLowState)
//                .afterDisp(85, robot.arm.armToDeposit)
//                .afterDisp(95, robot.wrist.turnWristZero)
//                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
//                .splineToConstantHeading(new Vector2d(49, -20), Math.toRadians(-30))
                .splineToConstantHeading(new Vector2d(40, -20), Math.toRadians(-30))
//                .splineToConstantHeading(new Vector2d(54, -20), Math.toRadians(0), robot.drive.depositVelConstraint, new ProfileAccelConstraint(-5,5))
//                .waitSeconds(0.5)
//                .setReversed(false)
//                .afterDisp(5, robot.arm.armToIdle)
//                .afterDisp(25, robot.lift.lowerLiftToGroundState)
//                .splineToConstantHeading(new Vector2d(0, -4), Math.toRadians(180))
//                .afterDisp(0, robot.wrist.turnWristNinety)
//                .afterDisp(0, robot.lift.lowerLiftToIdleState)
//                .afterDisp(40, intakePixelColorSensor(robot))
////                .afterDisp(40, robot.intake.intakePixelSecondPassPixelSpikeLeft)
//                .splineToConstantHeading(new Vector2d(-50, -4), Math.toRadians(180))
//                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeight)
////                .afterDisp(0, robot.intake.intakePixelSecondPassPixelSpikeLeft)
//                .splineToConstantHeading(new Vector2d(-59, -4), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
//                .waitSeconds(0.5)
//                .setReversed(true)
//                .afterDisp(15, robot.intake.intakeExtra)
//                .afterDisp(30, robot.lift.lowerLiftToGroundState)
//                .afterDisp(40, robot.depositor.bothDepositorsPickup)
////                .afterDisp(60,robot.intake.spitPixel)
//                .afterDisp(80, robot.lift.raiseLiftToMiddleState)
//                .afterDisp(85, robot.arm.armToDeposit)
////                .afterDisp(95, robot.wrist.turnWristOneEighty)
////                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
//                .splineToConstantHeading(new Vector2d(40, -20), Math.toRadians(-30))
//                .splineToConstantHeading(new Vector2d(54, -20), Math.toRadians(0), robot.drive.depositVelConstraint, new ProfileAccelConstraint(-5,5))
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
                .splineToLinearHeading(new Pose2d(-34, -12, Math.toRadians(180)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-36, -24), Math.toRadians(-90))
                .afterDisp(55, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-50, -4), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-50, -4, Math.toRadians(180)), Math.toRadians(180))
//                .waitSeconds(1.0)
//                .afterDisp(0, robot.intake.intakePixel)
                .splineToConstantHeading(new Vector2d(-57, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-56, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
//                .afterDisp(0, robot.intake.intakePixel)
                .splineToConstantHeading(new Vector2d(-56.25, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5, 5))
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.lift.lowerLiftToGroundState)
                .stopAndAdd(robot.depositor.bothDepositorsPickup)
                .setReversed(true)
//                .afterDisp(5, robot.intake.intakeExtra)
//                .afterDisp(100, robot.lift.lowerLiftToGroundState)
//                .afterDisp(110, robot.depositor.bothDepositorsPickup)
//                .afterDisp(120, robot.lift.raiseLiftAutoToLowState)
//                .afterDisp(100, robot.arm.armToDeposit)
//                .afterDisp(110, robot.wrist.turnWristOneEighty)
//                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .afterDisp(80, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(90, robot.arm.armToDeposit)
                .afterDisp(100, robot.wrist.turnWristOneEighty)
                .splineToConstantHeading(new Vector2d(40, -24), Math.toRadians(-30))
                .afterDisp(10, robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(54, -20), Math.toRadians(0), robot.drive.depositVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(false)
                .afterDisp(5, robot.arm.armToIdle)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .splineToConstantHeading(new Vector2d(0, -8), Math.toRadians(180))
                .afterDisp(0, robot.lift.lowerLiftToIdleState)
                .afterDisp(0, robot.wrist.turnWristNinety)
                .afterDisp(40, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-50, -4), Math.toRadians(180))
                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeight)
////                .afterDisp(0, robot.intake.intakePixelSecondTimeBlueRight)
                .splineToConstantHeading(new Vector2d(-57.5, -4), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.lift.lowerLiftToGroundState)
                .stopAndAdd(robot.depositor.bothDepositorsPickup)
//                .setReversed(true)
//                .afterDisp(10, robot.lift.lowerLiftToGroundState)
//                .afterDisp(40, robot.depositor.bothDepositorsPickup)
//                .afterDisp(80, robot.lift.raiseLiftToMiddleState)
//                .afterDisp(85, robot.arm.armToDeposit)
//                .afterDisp(95, robot.wrist.turnWristOneEighty)
//                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
//                .splineToConstantHeading(new Vector2d(51.5, -24), Math.toRadians(-45))
//                // move to position to drop purple pixel - relative to where robot stopped after seeing the center spike
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_RedLeft)
                .setReversed(true)
                .afterDisp(47, robot.drawbridge.drawBridgeUp)
                .splineToLinearHeading(new Pose2d(-22, -36, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-45, -4, Math.toRadians(180)), Math.toRadians(70))
                .setReversed(false)
//                .afterDisp(30, robot.intake.intakeFirstPixelRightSpikeRedSideLeft)
                .afterDisp(30, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-57, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-56, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
//                .afterDisp(0, robot.intake.intakeFirstPixelRightSpikeRedSideLeft)
                .afterDisp(0, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-56, -4), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5, 5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(5, robot.lift.lowerLiftToGroundState)
                .afterDisp(40, robot.depositor.bothDepositorsPickup)
//                .afterDisp(45, robot.intake.spitPixel)
                .afterDisp(60, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(65, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(51, -28), Math.toRadians(-45))
                .waitSeconds(0.5)
                .setReversed(false)
                .afterDisp(5, robot.wrist.turnWristNinety)
                .afterDisp(5, robot.arm.armToIdle)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .splineToConstantHeading(new Vector2d(0, -8), Math.toRadians(180))
                .afterDisp(0, robot.lift.lowerLiftToIdleState)
                .afterDisp(0, robot.wrist.turnWristNinety)
                .afterDisp(40, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-50, -5), Math.toRadians(180))
                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeight)
//                .afterDisp(0, robot.intake.intakePixelSecondTime)
                .afterDisp(0, intakePixelColorSensor(robot))
                .splineToConstantHeading(new Vector2d(-56, -5), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(10, robot.lift.lowerLiftToGroundState)
                .afterDisp(40, robot.depositor.bothDepositorsPickup)
                .afterDisp(60, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(65, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(51, -20), Math.toRadians(-45))
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
