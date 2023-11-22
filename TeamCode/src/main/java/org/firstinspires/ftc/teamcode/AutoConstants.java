package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoConstants {

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
    public static Vector2d vRedStack_Inner, vRedStack_Center, vRedStack_Outer, vBlueStack_Inner, vBlueStack_Center, vBlueStack_Outer,
            vRedLeftSpike_Left,vRedLeftSpike_Center, vRedLeftSpike_Right, vRedRightSpike_Left, vRedRightSpike_Center, vRedRightSpike_Right,
            vBlueRightSpike_Left, vBlueRightSpike_Center, vBlueRightSpike_Right, vBlueLeftSpike_Left, vBlueLeftSpike_Center, vBlueLeftSpike_Right;

    // Backstage positions
    public static double FIELD_BACKSTAGE_X = 2.0 * TILE_CENTER_TO_CENTER + TILE_TEETH / 2.0 + 0.5;
    public static double FIELD_BACKDROP_X = TILE_CENTER_TO_CENTER + TILE_CENTER_TO_EDGE;

    // Backdrop April Tag Positions
//    public static Vector2d vRedBackdrop_Left = new Vector2d(FIELD_BACKDROP_X, -1.5 * TILE_CENTER_TO_CENTER + 6);
    public static Vector2d vRedBackdrop_Left = new Vector2d(36, -28);
    //    public static Vector2d vRedBackdrop_Center = new Vector2d(FIELD_BACKDROP_X, -1.5 * TILE_CENTER_TO_CENTER);
    public static Vector2d vRedBackdrop_Center = new Vector2d(36,-36);
    //    public static Vector2d vRedBackdrop_Right = new Vector2d(FIELD_BACKDROP_X, -1.5 * TILE_CENTER_TO_CENTER - 6);
    public static Vector2d vRedBackdrop_Right = new Vector2d(36, -40);

    public static Vector2d vBlueBackdrop_Left = new Vector2d(FIELD_BACKDROP_X, 1.5 * TILE_CENTER_TO_CENTER - 6);
    public static Vector2d vBlueBackdrop_Center = new Vector2d(FIELD_BACKDROP_X, 1.5 * TILE_CENTER_TO_CENTER);
    public static Vector2d vBlueBackdrop_Right = new Vector2d(FIELD_BACKDROP_X, 1.5 * TILE_CENTER_TO_CENTER + 6);

    // Important waypoints on the field
    public static Vector2d vRedClearStageGate = new Vector2d(TILE_CENTER_TO_CENTER / 2.0, -TILE_CENTER_TO_CENTER / 2.0);
    public static Vector2d vBlueClearStageGate = new Vector2d(TILE_CENTER_TO_CENTER / 2.0, TILE_CENTER_TO_CENTER / 2.0);

    // Robot dimensions. Will become handy to orient around waypoints
    static double robot_length = 14;
    static double robot_width = 12;

    // Starting positions
    public static Pose2d pStartingPose_RedLeft = new Pose2d(-1.5 * TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER + robot_length / 2, Math.toRadians(-90));
    public static Pose2d pStartingPose_RedRight = new Pose2d(0.5 * TILE_CENTER_TO_CENTER, -FIELD_BOUNDARY_FROM_CENTER + robot_length / 2, Math.toRadians(-90));
    public static Pose2d pStartingPose_BlueRight = new Pose2d(-1.5 * TILE_CENTER_TO_CENTER, FIELD_BOUNDARY_FROM_CENTER - robot_length / 2, Math.toRadians(90));
    public static Pose2d pStartingPose_BlueLeft = new Pose2d(0.5 * TILE_CENTER_TO_CENTER, FIELD_BOUNDARY_FROM_CENTER - robot_length / 2, Math.toRadians(90));

}
