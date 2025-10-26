package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class AutoDepositConfig {
    // Shooter and hood
    public static double SHOOT_TARGET_RPM = 250;
    public static double SHOOT_RPM_BAND = 100;
    public static double HOOD_BASE_POS = 0.52;
    public static double HOOD_PER_100RPM = 0.00;
    public static double HOOD_MIN = 0.00;
    public static double HOOD_MAX = 1.00;
    // Spindexer
    public static double SPINDEXER_INDEX_TIME_S = 1.30;
    public static boolean REQUIRE_RPM_AT_FLICK = true;

    // Verification
    public static double SHOT_VERIFY_WINDOW_S = 0.50;
    public static int REFIRE_MAX_ATTEMPTS = 1;
    public static int JIGGLE_MAX_CYCLES = 3;
    public static double JIGGLE_DELTA_UP = 0.10;
    public static double JIGGLE_DELTA_DOWN = 0.20;
    public static double JIGGLE_DWELL_S = 0.12;
    public static boolean RPM_DROP_USES_MOTOR_RPM = true;

    // Feeding
    public static double FEED_ONE_TIME_S = 0.45;
    public static double RPM_RECOVER_TIMEOUT_S = 1.2;
    public static int MAX_FEEDS = 4;

    // Timeouts
    public static double SPINUP_TIMEOUT_S = 2.5;
    public static double DRIVE_TIMEOUT_S = 4.0;
    public static double TOTAL_TIMEOUT_S = 15.0;

    // Fallback heading if no tag seen (deg)
    public static double FALLBACK_HEADING_RED_DEG = 0;
    public static double FALLBACK_HEADING_BLUE_DEG = 180;

    // In field coordinates
    public static double RED_SHOT_X = 100;
    public static double RED_SHOT_Y = 100;

    public static double BLUE_SHOT_X = 35;
    public static double BLUE_SHOT_Y = 108;
    public static Pose pickShootPose(boolean isRed, boolean isAudienceSide) {
        if (isRed && isAudienceSide) {
            return new Pose(100, 100, Math.toRadians(45));
        } else if (!isRed && isAudienceSide) {
            return new Pose(42, 105, Math.toRadians(135));
        } else if (isRed) {
            return new Pose(100, 100, Math.toRadians(45));
        } else {
            return new Pose(42, 105, Math.toRadians(135));
        }
    }
}
