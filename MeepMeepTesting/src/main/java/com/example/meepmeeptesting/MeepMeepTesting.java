package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    // Important field positions

    static double TILE_INSIDE_TO_INSIDE = 22.875;
    static double TILE_CENTER_TO_CENTER = 23.625;
    static double TILE_CENTER_TO_EDGE   = 23.25;
    static double TILE_TEETH            = 0.75;

    static double FIELD_BOUNDARY_FROM_CENTER = 2.0 * TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // White Pixel Stack positions
    static Vector2d vRedStack_Inner   = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,-FIELD_BOUNDARY_FROM_CENTER + 35.0);
    Vector2d vRedStack_Center  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,-FIELD_BOUNDARY_FROM_CENTER + 46.875);
    static Vector2d vRedStack_Outer   = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,-FIELD_BOUNDARY_FROM_CENTER + 58.75);

    Vector2d vBlueStack_Inner  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,FIELD_BOUNDARY_FROM_CENTER - 35.0);
    Vector2d vBlueStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,FIELD_BOUNDARY_FROM_CENTER - 46.875);
    Vector2d vBlueStack_Outer  = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5,FIELD_BOUNDARY_FROM_CENTER - 58.75);

    // Spike positions
    Vector2d vRedLeftSpike_Left   = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 6.0);
    Vector2d vRedLeftSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5);
    Vector2d vRedLeftSpike_Right  = new Vector2d(-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 6.0);

    Vector2d vRedRightSpike_Left   = new Vector2d(TILE_TEETH / 2.0 + 0.5,-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);
    Vector2d vRedRightSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5);
    Vector2d vRedRightSpike_Right  = new Vector2d(TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5,-TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 6.0);

    Vector2d vBlueRightSpike_Left   = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    Vector2d vBlueRightSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    Vector2d vBlueRightSpike_Right  = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    Vector2d vBlueLeftSpike_Left   = new Vector2d(TILE_TEETH / 2.0 + 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    Vector2d vBlueLeftSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    Vector2d vBlueLeftSpike_Right  = new Vector2d(TILE_CENTER_TO_CENTER-TILE_TEETH / 2.0 - 0.5,TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    // Backstage positions
    double FIELD_BACKSTAGE_X = 2.0 * TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5;
    static double FIELD_BACKDROP_X  = TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // Backdrop April Tag Positions
    Vector2d vRedBackdrop_Left   = new Vector2d(FIELD_BACKDROP_X,-1.5 * TILE_CENTER_TO_CENTER + 6);
    public static Vector2d vRedBackdrop_Center = new Vector2d(FIELD_BACKDROP_X,-1.5 * TILE_CENTER_TO_CENTER);
    Vector2d vRedBackdrop_Right  = new Vector2d(FIELD_BACKDROP_X,-1.5 * TILE_CENTER_TO_CENTER - 6);

    Vector2d vBlueBackdrop_Left   = new Vector2d(FIELD_BACKDROP_X,1.5 * TILE_CENTER_TO_CENTER - 6);
    Vector2d vBlueBackdrop_Center = new Vector2d(FIELD_BACKDROP_X,1.5 * TILE_CENTER_TO_CENTER);
    Vector2d vBlueBackdrop_Right  = new Vector2d(FIELD_BACKDROP_X,1.5 * TILE_CENTER_TO_CENTER + 6);


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
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





        Action trajectory =
                myBot.getDrive().actionBuilder(new Pose2d(-36,-61, Math.toRadians(-90)))

                        // CENTER PLACEMENT

                        // Replace prop with your yellow pixel (just push)
                        .lineToY(-30)
                        .waitSeconds(1)

                        // Goto Backdrop to place your purple pixel
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(vRedBackdrop_Center.x, vRedBackdrop_Center.y, Math.toRadians(0)), Math.toRadians(200))
//                        .splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(180)), Math.toRadians(0))
//                        .strafeTo(vRedBackdrop_Center)

//                        .splineTo(vRedBackdrop_Center, Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(-36,-12,Math.toRadians(0)),Math.toRadians(0))
//                        .lineToX(48)
                        .waitSeconds(1)

                        // Goto stack and collect 2 white pixels
                        .setReversed(false)
                        .setTangent(Math.toRadians(180))
//                        .strafeTo(new Vector2d(12, -12))
                        .setTangent(-180)
                        .splineToConstantHeading(vRedStack_Outer, Math.toRadians(180))
                        .waitSeconds(1)

                        // Goto Backstage and drop 2 white pixels
                        .setReversed(false)
                        .lineToXConstantHeading(24)
                        .setTangent(Math.toRadians(0))
                        .strafeTo(vRedBackdrop_Center)
                        .waitSeconds(1)

                        // go to alliance partner's robot and collect their pixels

                        // drop their yellow pixel to the right spike

                        // place their purple pixel to the backdrop

                        // park

                        .build();

        myBot.runAction(trajectory);


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}