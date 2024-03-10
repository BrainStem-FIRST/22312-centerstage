package org.firstinspires.ftc.teamcode.AutoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// This class contains the BLUE RIGHT AUTO Trajectories and Actions including Cycle
// Whether the Cycle trajectories run is determined by the child classes and acted upon
// by the AutoAbstractOpMode class.
public abstract class AutoAbstractBR extends AutoAbstractOpMode {

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
                .setReversed(false)

                // Turn heading towards right spike
                .setTangent(Math.toRadians(35))
                .lineToYLinearHeading(30, Math.toRadians(25))

                // Drop yellow pixel in position
                .stopAndAdd(robot.intake.spitPixel)

                // Goto Backdrop to place your yellow pixel
                .setReversed(true)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d((-constants.TILE_CENTER_TO_CENTER*2.25) + 12, (constants.TILE_CENTER_TO_CENTER / 2.0) + 12, Math.toRadians(180.00001)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x, constants.vBlueClearStageGate.y + 12, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d((constants.vBlueBackdrop_Left.x) + 12, constants.vBlueBackdrop_Left.y + 12, Math.toRadians(-180)), Math.toRadians(90))

                .build();
    }

    @Override
    public Action traj_center(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                // move to position to drop purple pixel - relative to where robot stopped after seeing the center spike
                .lineToY(robot.drive.pose.position.y - 8.0)

                // Drops purple pixel at spike. May need to set ds negative so the drawbridge lifts
                .afterDisp(0,robot.drawbridge.drawBridgeUp)

                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-50, constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))

                // Start rolling 1 sec before arrival relative to above splineTo. Automatically stops after x seconds
                .afterTime(-1,robot.intake.intakePixel)

                // May need to jiggle here to ensure picking up white pixel
                .waitSeconds(1.5)

                // Secure the cargo
                .afterTime(0, robot.lift.lowerLiftToGroundState)
                .afterTime(1.5, robot.depositor.bothDepositorsPickup)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x, constants.vBlueClearStageGate.y, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vBlueBackdrop_Center.x, constants.vBlueBackdrop_Center.y, Math.toRadians(-180)), Math.toRadians(90))

                .build();
    }

    @Override
    public Action traj_right(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                // go backwards
                .setReversed(true)

                .setTangent(135)
                .lineToX(constants.vBlueRightSpike_Right.x)
//                .splineTo(new Vector2d(constants.vRedLeftSpike_Left.x, constants.vRedLeftSpike_Left.y - 4.0), Math.toRadians(90))
//                .lineToY(constants.vRedLeftSpike_Left.y + constants.robot_length / 2.0)

                .stopAndAdd(robot.intake.spitPixel)

                .endTrajectory()
                .setReversed(true)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-constants.TILE_CENTER_TO_CENTER*2.25, constants.TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(180))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.vBlueClearStageGate.x-8, constants.vBlueClearStageGate.y, Math.toRadians(180)), Math.toRadians(0)) // added delta to x so we don't un-score partner's pixel
                .splineToLinearHeading(new Pose2d(constants.vBlueBackdrop_Right.x-4, constants.vBlueBackdrop_Right.y, Math.toRadians(-180)), Math.toRadians(90))

                .build();
    }


    @Override
    public Action cycle(BrainSTEMRobotA robot) {
        return robot.drive.actionBuilder(robot.drive.pose)
                .setTangent(-90)

                // Get ready for next pickup prior to going under the truss
                .afterTime(0, robot.wrist.turnWristNinety)
                .afterTime(0.5, robot.arm.armToIdle)
                .afterTime(1.5, robot.lift.lowerLiftToIdleState)

                .splineToLinearHeading(new Pose2d(-50, constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(-180)), Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-57, constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(-180)), Math.toRadians(-180), robot.drive.pickupVelConstraint, new ProfileAccelConstraint(-5,5))

                // Start rolling 1 sec before arrival relative to above splineTo. Automatically stops after x seconds
                .afterTime(-1,robot.intake.intakePixel)

                // May need to jiggle here to ensure picking up white pixel
                .waitSeconds(2.0)

                // Secure the cargo
                .afterTime(0, robot.lift.lowerLiftToGroundState)
                .afterTime(1.5, robot.depositor.bothDepositorsPickup)


                .setTangent(0)
                .splineToLinearHeading(new Pose2d(constants.TILE_CENTER_TO_CENTER / 2+6, constants.TILE_CENTER_TO_CENTER / 2, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(constants.vBlueBackdrop_Center.x+2, constants.vBlueBackdrop_Center.y-2, Math.toRadians(180)), Math.toRadians(90))
                .build();
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
        return Orientation.RIGHT;
    }
}