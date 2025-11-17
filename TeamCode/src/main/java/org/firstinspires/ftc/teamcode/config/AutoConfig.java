package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class AutoConfig {
    // All in pedropathing coordinates
    // Field/robot constants

    public static boolean isRed = true;
    public static boolean isAudienceSide = true;

    // Start poses
    public static Pose START_RED_AUDIENCE = new Pose(88,7.5, Math.toRadians(90));
    public static Pose START_BLUE_AUDIENCE = new Pose(56,7.5, Math.toRadians(90));
    public static Pose START_RED_DEPOT = new Pose(117.5,131.2,Math.toRadians(-142));
    public static Pose START_BLUE_DEPOT = new Pose(27.29,131.2, Math.toRadians(-36));

    // Vision
    public static int APRIL_TAG_BLUE = 20;
    public static int APRIL_TAG_RED = 24;
    public static int APRIL_TAG_GPP = 21;
    public static int APRIL_TAG_PGP = 22;
    public static int APRIL_TAG_PPG = 23;
}
