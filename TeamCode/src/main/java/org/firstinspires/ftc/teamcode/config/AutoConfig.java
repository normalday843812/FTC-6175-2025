package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class AutoConfig {
    // All in pedropathing coordinates
    // Field/robot constants
    private static final double ROBOT_LENGTH = 15.25;
    private static final double ROBOT_WIDTH = 16;
    private static final double MAX_SIZE = Math.min(ROBOT_LENGTH, ROBOT_WIDTH);

    public static boolean isRed = true;
    public static boolean isAudienceSide = true;

    // Start poses
    public static Pose START_RED_AUDIENCE = new Pose(88,7.5, Math.toRadians(90));
    public static Pose START_BLUE_AUDIENCE = new Pose(56,7.5, Math.toRadians(90));
    public static Pose START_RED_DEPOT = new Pose(117.5,131.2,Math.toRadians(-142));
    public static Pose START_BLUE_DEPOT = new Pose(27.29,131.2, Math.toRadians(-36));

    // Poses
    public static Pose LEAVE_LAUNCH_LINE_RED = new Pose(88,58, Math.toRadians(0));
    public static Pose LEAVE_LAUNCH_LINE_BLUE = new Pose(56,58, Math.toRadians(180));


    // Vision
    public static int APRIL_TAG_BLUE = 20;
    public static int APRIL_TAG_RED = 24;
    public static int APRIL_TAG_GPP = 21;
    public static int APRIL_TAG_PGP = 22;
    public static int APRIL_TAG_PPG = 23;
    // Restrictions
    public static final double PADDING = 1;
    public static final double MAXIMUM_BLUE_X = (144.0 / 2) - (MAX_SIZE * Math.cos(Math.toRadians(45))) - PADDING;
    public static final double MINIMUM_BLUE_X = MAX_SIZE / 2;
    public static final double MINIMUM_RED_X = (144.0 / 2) + (MAX_SIZE * Math.cos(Math.toRadians(45))) - PADDING;
    public static final double MAXIMUM_RED_X = 144.0 - (MAX_SIZE / 2);
    public static final double MAXIMUM_Y = 144.0 - (MAX_SIZE / 2);
    public static final double MINIMUM_Y = MAX_SIZE / 2;
}
