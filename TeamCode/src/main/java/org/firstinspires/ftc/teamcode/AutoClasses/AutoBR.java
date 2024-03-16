package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous(name="Robot: Blue-Right Auto", group="Robot")
public class AutoBR extends AutoAbstractOpMode {

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
        return robot.drive.actionBuilder(robot.drive.pose)
                // go forwards
                .setReversed(true)

                .splineToLinearHeading(new Pose2d(-34, 24, Math.toRadians(0)), Math.toRadians(0), robot.drive.pickupVelConstraint)
//
//                // Turn heading towards right spike
//                .setTangent(Math.toRadians(35))
//                .lineToYLinearHeading(30, Math.toRadians(25))
//
//                // Drop yellow pixel in position
                .stopAndAdd(robot.drawbridge.drawBridgeUp)
//
//                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(180))
                .lineToX(robot.drive.pose.position.x - 10)
//                .setTangent(Math.toRadians(-135))
//                .splineToLinearHeading(new Pose2d((-constants.TILE_CENTER_TO_CENTER*2.25) + 12, (constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180.00001)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-45, 12, Math.toRadians(180)), Math.toRadians(0))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x-8, constants.vBlueClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel

                .splineToConstantHeading(new Vector2d(30, 44), Math.toRadians(60))
                .stopAndAdd(robot.intake.intakeSafeAutos)
                .stopAndAdd(robot.lift.liftToGroundStateSafeAuto)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.depositor.bothDepositorPickupSafe)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(41, 44))
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                // move to position to drop purple pixel - relative to where robot stopped after seeing the center spike
                .lineToY(robot.drive.pose.position.y - 6.0)

                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                .setReversed(true)
                .setTangent(90)
                .lineToY(robot.drive.pose.position.y - 5)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-45, 12, Math.toRadians(180)), Math.toRadians(0))

                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, 40), Math.toRadians(60))

                .stopAndAdd(robot.intake.intakeSafeAutos)
                .stopAndAdd(robot.lift.liftToGroundStateSafeAuto)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.depositor.bothDepositorPickupSafe)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(41, 40))
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25 - 12, (constants.TILE_CENTER_TO_CENTER / 2.0) +  12, Math.toRadians(180.00001)), Math.toRadians(180))
//                .stopAndAdd(robot.intake.intakePixel)
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x-8, constants.vBlueClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
//                .splineToLinearHeading(new Pose2d(constants.vBlueBackdrop_Center.x - 12, constants.vBlueBackdrop_Center.y + 12, Math.toRadians(-180)), Math.toRadians(90))

                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-50, 34, Math.toRadians(180)), Math.toRadians(180))

//                .setTangent(135)
//                .lineToX(constants.vBlueRightSpike_Right.x)
////                .splineTo(new Vector2d(constants.vRedLeftSpike_Left.x, constants.vRedLeftSpike_Left.y - 4.0), Math.toRadians(90))
////                .lineToY(constants.vRedLeftSpike_Left.y + constants.robot_length / 2.0)
//
                .stopAndAdd(robot.drawbridge.drawBridgeUp)

                .setReversed(true)

                .splineToConstantHeading(new Vector2d(-45, 12), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(30, 34), Math.toRadians(60))


                .stopAndAdd(robot.intake.intakeSafeAutos)
                .stopAndAdd(robot.lift.liftToGroundStateSafeAuto)
                .stopAndAdd(robot.intake.intakeExtra)
                .stopAndAdd(robot.depositor.bothDepositorPickupSafe)
                .stopAndAdd(robot.lift.raiseLiftAutoToLowState)
                .stopAndAdd(robot.arm.armToDeposit)
                .strafeToConstantHeading(new Vector2d(41, 33))
                .stopAndAdd(robot.depositor.bothDepositorsDeposit)
//
//                .endTrajectory()
//                .setReversed(true)
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, (constants.TILE_CENTER_TO_CENTER / 2.0), Math.toRadians(180.00001)), Math.toRadians(180))
//                .setTangent(0)
//                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x-8, constants.vBlueClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
//                .splineToLinearHeading(new Pose2d(constants.vBlueBackdrop_Right.x-4, constants.vBlueBackdrop_Right.y + 12, Math.toRadians(-180)), Math.toRadians(90))

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
                .splineToConstantHeading(new Vector2d(robot.drive.pose.position.x - 4, 20),Math.toRadians(90))                .build();
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
                .strafeToConstantHeading(new Vector2d(50, -18))
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