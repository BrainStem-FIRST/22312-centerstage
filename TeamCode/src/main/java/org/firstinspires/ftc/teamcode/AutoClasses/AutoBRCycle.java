package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;

@Config
@Autonomous(name="Robot: Blue-Right Auto Cycle", group="Robot")
public class AutoBRCycle extends AutoAbstractOpModeCycle {

    AutoConstants constants;

    @Override
    public Pose2d startPose() {
        return constants.pStartingPose_BlueRight;
    }

    @Override
    public Action traj_init(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go backwards
                .setReversed(true)

                // Move close enough to center spike
                .lineToY(30.5)

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
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go forwards
                .setReversed(true)
                .afterDisp(47, robot.drawbridge.drawBridgeUpBlue)
                .splineToLinearHeading(new Pose2d(-38, 25, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.5)
//                .afterDisp(80, robot.intake.intakeFirstPixelBlueSpikeCenter)
//                .afterDisp(55, robot.intake.intakeFirstPixelBlueSpikeCenter)
//                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-58, 15, Math.toRadians(180)), Math.toRadians(-70))
                .afterDisp(0, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-68.5, 15), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-67, 15), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .afterDisp(0, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-68.25, 17), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(0, robot.lift.lowerLiftToGroundState)
                .afterDisp(5, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristZero)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(39, 45), Math.toRadians(55))
                .waitSeconds(0.5)
                .setReversed(false)
                .afterDisp(5, robot.arm.armToIdle)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(180))
                .afterDisp(0, robot.lift.lowerLiftToIdleState)
                .afterDisp(0, robot.wrist.turnWristNinety)
                .afterDisp(40, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(180))
                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeight)
                .afterDisp(0, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-69, 15), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(0, robot.lift.lowerLiftToGroundState)
                .afterDisp(5, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftToMiddleState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(39, 45), Math.toRadians(55))
                .waitSeconds(0.5)
                .build();

    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go backwards
                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(180)), Math.toRadians(90))
                .afterDisp(55, robot.drawbridge.drawBridgeUpBlue)
//                .splineToLinearHeading(new Pose2d(-43, 28, Math.toRadians(180)), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-35, 20), Math.toRadians(-90))
                .setReversed(true)
//                .afterDisp(, robot.intake.intakeFirstPixelBlueSpikeCenter)
//                .splineToConstantHeading(new Vector2d(-58, 17), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-50, -4, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-58, 17, Math.toRadians(180)), Math.toRadians(180))
//                .waitSeconds(1.0)
                .afterDisp(20, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-68.5, 17), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-67, 17), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .afterDisp(0, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-68.25, 17), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(0, robot.lift.lowerLiftToGroundState)
                .afterDisp(5, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(39, 40), Math.toRadians(55))
                .waitSeconds(0.5)
                .setReversed(false)
                .afterDisp(5, robot.arm.armToIdle)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(180))
                .afterDisp(0, robot.lift.lowerLiftToIdleState)
                .afterDisp(0, robot.wrist.turnWristNinety)
                .afterDisp(40, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-58, 17), Math.toRadians(180))
                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeightBlue)
                .afterDisp(0, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-69.5, 17), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(0, robot.lift.lowerLiftToGroundState)
                .afterDisp(10, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftToMiddleState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(39, 40), Math.toRadians(55))
                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(constants.pStartingPose_BlueRight)
                // go backwards
                .setReversed(true)
                .afterDisp(20, robot.drawbridge.drawBridgeUpBlue)
                .splineToLinearHeading(new Pose2d(-46, 34, Math.toRadians(180)), Math.toRadians(-90))
//                .afterDisp(80, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .afterDisp(55, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(180))
                .afterDisp(0, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-68.5, 15), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .splineToConstantHeading(new Vector2d(-67, 15), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .afterDisp(0, robot.intake.intakeFirstPixelBlueSpikeCenter)
                .splineToConstantHeading(new Vector2d(-68.25, 17), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(0, robot.lift.lowerLiftToGroundState)
                .afterDisp(5, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftAutoToLowState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(39, 35), Math.toRadians(55))
                .waitSeconds(0.5)
                .setReversed(false)
                .afterDisp(5, robot.arm.armToIdle)
                .afterDisp(40, robot.lift.lowerLiftToGroundState)
                .splineToConstantHeading(new Vector2d(0, 12), Math.toRadians(180))
                .afterDisp(0, robot.lift.lowerLiftToIdleState)
                .afterDisp(0, robot.wrist.turnWristNinety)
                .afterDisp(40, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-58, 15), Math.toRadians(180))
                .afterDisp(0, robot.drawbridge.setDrawBridgeFourthHeight)
                .afterDisp(0, robot.intake.intakePixelSecondTime)
                .splineToConstantHeading(new Vector2d(-69, 15), Math.toRadians(180), robot.drive.secondPickupVelConstraint, new ProfileAccelConstraint(-5,5))
                .waitSeconds(0.5)
                .setReversed(true)
                .afterDisp(0, robot.lift.lowerLiftToGroundState)
                .afterDisp(5, robot.depositor.bothDepositorsPickup)
                .afterDisp(80, robot.lift.raiseLiftToMiddleState)
                .afterDisp(85, robot.arm.armToDeposit)
                .afterDisp(95, robot.wrist.turnWristOneEighty)
                .afterDisp(120,robot.depositor.bothDepositorsDeposit)
                .splineToConstantHeading(new Vector2d(39, 35), Math.toRadians(55))
                .waitSeconds(0.5)
                .build();
    }

    @Override
    public Action cycle(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(-90)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(-180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER * 2.5, constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(-180)), Math.toRadians(-180))
                .waitSeconds(2.0)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(constants.vBlueBackdrop_Center.x+2, constants.vBlueBackdrop_Center.y-2, Math.toRadians(180)), Math.toRadians(90))
                .build();
    }

    @Override
    public Action parking_traj(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(robot.drive.pose.position.x - 4, 20),Math.toRadians(90))
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
        return Alliance.BLUE;
    }

    @Override
    public Orientation orientation() {
        return Orientation.RIGHT;
    }
}