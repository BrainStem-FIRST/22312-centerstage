package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.sun.tools.javac.util.Convert;


public class MeepMeepTesting {

    // Robot dimensions. Will become handy to orient around waypoints
    static double robot_length = 15.75;
    static double robot_width = 14;

    static double IN_TO_MM = 25.4;  // conversion multiplier from inch to millimeter
    static double DISTANCE_BETWEEN_DISTSENSORS = 10;    // distance between two sensors in inches

    // Important field positions

    static double TILE_INSIDE_TO_INSIDE = 22.875;
    static double TILE_CENTER_TO_CENTER = 23.625;
    static double TILE_CENTER_TO_EDGE = 23.25;
    static double TILE_TEETH = 0.75;

    static double FIELD_BOUNDARY_FROM_CENTER = 2.0 * TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // White Pixel Stack positions
    public static Vector2d vRedStack_Inner = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, -FIELD_BOUNDARY_FROM_CENTER + 35.0);
    public static Vector2d vRedStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, -FIELD_BOUNDARY_FROM_CENTER + 46.875);
    public static Vector2d vRedStack_Outer = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, -FIELD_BOUNDARY_FROM_CENTER + 58.75);

    public static Vector2d vBlueStack_Inner = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, FIELD_BOUNDARY_FROM_CENTER - 35.0);
    public static Vector2d vBlueStack_Center = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, FIELD_BOUNDARY_FROM_CENTER - 46.875);
    public static Vector2d vBlueStack_Outer = new Vector2d(-FIELD_BOUNDARY_FROM_CENTER + 1.5, FIELD_BOUNDARY_FROM_CENTER - 58.75);

    // Spike positions
    public static Vector2d vRedLeftSpike_Left = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);  /// was 10.0
    public static Vector2d vRedLeftSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5);
    public static Vector2d vRedLeftSpike_Right = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);

    public static Vector2d vRedRightSpike_Left = new Vector2d(TILE_TEETH / 2.0 + 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);
    public static Vector2d vRedRightSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5);
    public static Vector2d vRedRightSpike_Right = new Vector2d(TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, -TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 6.0);

    public static Vector2d vBlueRightSpike_Left = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_TEETH/2.0 - 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    public static Vector2d vBlueRightSpike_Center = new Vector2d(-1.5 * TILE_CENTER_TO_CENTER, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    public static Vector2d vBlueRightSpike_Right = new Vector2d(-TILE_CENTER_TO_CENTER - TILE_CENTER_TO_EDGE + 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    public static Vector2d vBlueLeftSpike_Left = new Vector2d(TILE_CENTER_TO_CENTER - TILE_TEETH / 2.0 - 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);
    public static Vector2d vBlueLeftSpike_Center = new Vector2d(0.5 * TILE_CENTER_TO_CENTER, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5);
    public static Vector2d vBlueLeftSpike_Right = new Vector2d(TILE_TEETH / 2.0 + 0.5, TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 6.0);

    // Backstage X Positions
    public static double FIELD_BACKSTAGE_X = 2.0 * TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5;
    public static double FIELD_BACKDROP_X = 2.5 * TILE_CENTER_TO_CENTER;

    // Backdrop Y Positions
    public static double FIELD_RED_BACKDROP_LEFT_Y   = -1.5 * TILE_CENTER_TO_CENTER + 6.0;
    public static double FIELD_RED_BACKDROP_CENTER_Y = -1.5 * TILE_CENTER_TO_CENTER;
    public static double FIELD_RED_BACKDROP_RIGHT_Y  = -1.5 * TILE_CENTER_TO_CENTER - 6.0;

    public static double FIELD_BLUE_BACKDROP_LEFT_Y   = 1.5 * TILE_CENTER_TO_CENTER + 6.0;
    public static double FIELD_BLUE_BACKDROP_CENTER_Y = 1.5 * TILE_CENTER_TO_CENTER;
    public static double FIELD_BLUE_BACKDROP_RIGHT_Y  = 1.5 * TILE_CENTER_TO_CENTER - 6.0;

    // Trajectory ending positions (takes into account the distance of robot from the backdrop)
    public static double MAX_DISTANCE_BEFORE_CRASH = 8.0;   // inches.  Adjust this value to determine robot's position before backdrop
    public static double BACKDROP_X_WITH_FOV = FIELD_BACKDROP_X - 12 - robot_length/2.0;    // Adjust value between backdrop and front of robot where it can still see AprilTags

    public static Vector2d vRedBackdrop_Left = new Vector2d(BACKDROP_X_WITH_FOV, FIELD_RED_BACKDROP_LEFT_Y);
    //    public static Vector2d vRedBackdrop_Left = new Vector2d(36, -28);
    public static Vector2d vRedBackdrop_Center = new Vector2d(BACKDROP_X_WITH_FOV, FIELD_RED_BACKDROP_CENTER_Y);
    //    public static Vector2d vRedBackdrop_Center = new Vector2d(36, -40);
    public static Vector2d vRedBackdrop_Right = new Vector2d(BACKDROP_X_WITH_FOV, FIELD_RED_BACKDROP_RIGHT_Y);
//    public static Vector2d vRedBackdrop_Right = new Vector2d(36, -40);

    public static Vector2d vBlueBackdrop_Left = new Vector2d(BACKDROP_X_WITH_FOV, FIELD_BLUE_BACKDROP_LEFT_Y);
    public static Vector2d vBlueBackdrop_Center = new Vector2d(BACKDROP_X_WITH_FOV, FIELD_BLUE_BACKDROP_CENTER_Y);
    public static Vector2d vBlueBackdrop_Right = new Vector2d(BACKDROP_X_WITH_FOV, FIELD_BLUE_BACKDROP_RIGHT_Y);


    // Important waypoints on the field
    public static Vector2d vRedClearStageGate = new Vector2d(TILE_CENTER_TO_CENTER / 2.0, -TILE_CENTER_TO_CENTER / 2.0 );
    public static Vector2d vBlueClearStageGate = new Vector2d(TILE_CENTER_TO_CENTER / 2.0, TILE_CENTER_TO_CENTER / 2.0 );

    // Starting positions
    public static Pose2d pStartingPose_RedLeft  = new Pose2d(-1.5 * TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER + robot_length / 2.0, Math.toRadians(-90));
    public static Pose2d pStartingPose_RedRight = new Pose2d(0.5 * TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER + robot_length / 2.0, Math.toRadians(-90));
    public static Pose2d pStartingPose_BlueRight = new Pose2d(-1.5 * TILE_CENTER_TO_CENTER, FIELD_BOUNDARY_FROM_CENTER - robot_length / 2.0, Math.toRadians(90));
    public static Pose2d pStartingPose_BlueLeft  = new Pose2d(0.5 * TILE_CENTER_TO_CENTER, FIELD_BOUNDARY_FROM_CENTER - robot_length / 2.0, Math.toRadians(90));

    // This distance is for fine tuning. Start with the trajectory ending position, but it can be adjusted independently for fine tuning.
    // For instance, trajectory brings the robot close enough using MAX_DISTANCE_BEFORE_CRASH, and target distance slowly moves the robot a little more to the backdrop.
    public static double targetDistance = 180.00; // in MM (empirical value was 200mm)



    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .setDimensions(robot_width, robot_length)
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
                myBot.getDrive().actionBuilder(new Pose2d(50,-40, Math.toRadians(-150)))
                        .setTangent(90)
                        .splineToLinearHeading(new Pose2d(TILE_CENTER_TO_CENTER,-TILE_CENTER_TO_CENTER/2, Math.toRadians(-180)), Math.toRadians(-180))
                        .splineToLinearHeading(new Pose2d(-TILE_CENTER_TO_CENTER*2.5, -TILE_CENTER_TO_CENTER/2, Math.toRadians(-180)), Math.toRadians(-180))
                        .build();

        myBot.runAction(trajectory);


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}