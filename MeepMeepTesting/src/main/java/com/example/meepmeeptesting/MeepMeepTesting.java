package com.example.meepmeeptesting;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // Important field positions

    static double IN_TO_MM = 25.4;  // conversion multiplier from inch to millimeter
    static double DISTANCE_BETWEEN_DISTSENSORS = 10;    // distance between two sensors in inches

    // Important field positions

    static double TILE_INSIDE_TO_INSIDE = 22.875;
    static double TILE_CENTER_TO_CENTER = 23.625;
    static double TILE_CENTER_TO_EDGE = 23.25;
    static double TILE_TEETH = 0.75;

    public boolean isOrientationLEFT, orientationIsSet, isAllianceRed, allianceIsSet, programConfirmation;

    static double FIELD_BOUNDARY_FROM_CENTER = 2.0 * TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // White Pixel Stack positions
    public static Vector2d vRedStack_Inner = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, -FIELD_BOUNDARY_FROM_CENTER + 35.0);
    public static Vector2d vRedStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, -FIELD_BOUNDARY_FROM_CENTER + 46.875);
    public static Vector2d vRedStack_Outer = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, -FIELD_BOUNDARY_FROM_CENTER + 58.75);

    public static Vector2d vBlueStack_Inner = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, FIELD_BOUNDARY_FROM_CENTER - 35.0);
    public static Vector2d vBlueStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, FIELD_BOUNDARY_FROM_CENTER - 46.875);
    public static Vector2d vBlueStack_Outer = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, FIELD_BOUNDARY_FROM_CENTER - 58.75);

    public static Vector2d vRedLeftSpike_Left = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);
    public static Vector2d vRedLeftSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5);
    public static Vector2d vRedLeftSpike_Right = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 9.0);

    public static Vector2d vRedRightSpike_Left = new Vector2d(TILE_TEETH / 2.0 + 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);
    public static Vector2d vRedRightSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5);
    public static Vector2d vRedRightSpike_Right = new Vector2d(TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);

    public static Vector2d vBlueRightSpike_Left = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    public static Vector2d vBlueRightSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    public static Vector2d vBlueRightSpike_Right = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    public static Vector2d vBlueLeftSpike_Left = new Vector2d(TILE_TEETH / 2.0 + 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    public static Vector2d vBlueLeftSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    public static Vector2d vBlueLeftSpike_Right = new Vector2d(TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    // Backstage positions
    public static double FIELD_BACKSTAGE_X = 2.0 * TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5;
    public static double FIELD_BACKDROP_X = TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // Backdrop April Tag Positions
//    public static Vector2d vRedBackdrop_Left = new Vector2d(FIELD_BACKDROP_X, -1.5 * TILE_CENTER_TO_CENTER + 6);
    public static Vector2d vRedBackdrop_Left = new Vector2d(36, -28);
    //        public static Vector2d vRedBackdrop_Center = new Vector2d(FIELD_BACKDROP_X, -1.5 * TILE_CENTER_TO_CENTER);
    public static Vector2d vRedBackdrop_Center = new Vector2d(20,-40);
    //    public static Vector2d vRedBackdrop_Right = new Vector2d(FIELD_BACKDROP_X, -1.5 * TILE_CENTER_TO_CENTER - 6);
    public static Vector2d vRedBackdrop_Right = new Vector2d(36, -40);

    public static Vector2d vBlueBackdrop_Left = new Vector2d(FIELD_BACKDROP_X, 1.5 * TILE_CENTER_TO_CENTER - 6);
    public static Vector2d vBlueBackdrop_Center = new Vector2d(FIELD_BACKDROP_X, 1.5 * TILE_CENTER_TO_CENTER);
    public static Vector2d vBlueBackdrop_Right = new Vector2d(FIELD_BACKDROP_X, 1.5 * TILE_CENTER_TO_CENTER + 6);

    // Important waypoints on the field
    public static Vector2d vRedClearStageGate = new Vector2d(TILE_CENTER_TO_CENTER / 2.0, -TILE_CENTER_TO_CENTER / 2.0);
    public static Vector2d vBlueClearStageGate = new Vector2d(TILE_CENTER_TO_CENTER / 2.0, TILE_CENTER_TO_CENTER / 2.0);

    // Robot dimensions. Will become handy to orient around waypoints
    static double robot_length = 15.75;
    static double robot_width = 14;

    // Starting positions
    public static Pose2d pStartingPose_RedLeft = new Pose2d(-1.5 * TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER + robot_length / 2, Math.toRadians(-90));
    public static Pose2d pStartingPose_RedRight = new Pose2d(0.5 * TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER + robot_length / 2, Math.toRadians(-90));
    public static Pose2d pStartingPose_BlueRight = new Pose2d(-1.5 * TILE_CENTER_TO_CENTER, FIELD_BOUNDARY_FROM_CENTER - robot_length / 2, Math.toRadians(90));
    public static Pose2d pStartingPose_BlueLeft = new Pose2d(0.5 * TILE_CENTER_TO_CENTER, FIELD_BOUNDARY_FROM_CENTER - robot_length / 2, Math.toRadians(90));

    public static int targetDistance = 100; //in MM

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(12,14)
                .build();



/********* Example **********
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .lineToX(20)
                .turn(Math.toRadians(90))
                .lineToY(30)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(90))
                .lineToY(0)
                .turn(Math.toRadians(90))
                .build());
*******************************/

        double robot_length = 14;

        Action trajectory =
                myBot.getDrive().actionBuilder(pStartingPose_RedLeft)
                        .setReversed(true)

                        // Go to position to drop yellow pixel (this is a little next to the team prop, not pushing it)
                        .lineToYSplineHeading(vRedLeftSpike_Right.y, Math.toRadians(0))
                        .endTrajectory()
                        .lineToX(vRedLeftSpike_Right.x - robot_length / 2)

                        // Discontinue trajectory
                        .endTrajectory()
                        .setReversed(true)

                        // Goto Backdrop to place your purple pixel
                        .setTangent(Math.toRadians(135))
                        .splineToLinearHeading(new Pose2d(-TILE_CENTER_TO_CENTER, -TILE_CENTER_TO_CENTER / 2.0, Math.toRadians(180.00001)), Math.toRadians(0))
                        .splineTo(vRedClearStageGate, Math.toRadians(0))
                        .splineTo(vRedBackdrop_Right, Math.toRadians(0))
                        .build();
        myBot.runAction(trajectory);


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}