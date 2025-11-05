package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoUnifiedConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Heading PD
    public static double HEADING_KP = 1.0;
    public static double HEADING_KD = 0.02;
    public static double HEADING_MAX_ROT = 0.7;
    public static double HEADING_DEADBAND_DEG = 0.5;

    // Field translation
    public static double DRIVE_APPROACH_GAIN = 0.06;
    public static double DRIVE_MAX_VEL = 0.95;
    public static double DRIVE_STOP_DIST_IN = 1.25;

    // Timeouts
    public static double DEFAULT_TIMEOUT_S = 4.0;
    public static double PATH_TIMEOUT_TO_GOAL_S = 4.5;
    public static double PATH_TIMEOUT_TO_INTAKE_S = 5.0;

    // Pattern seek
    public static long PATTERN_STABLE_HOLD_MS = ShooterYawConfig.STABLE_HOLD_MS_DEFAULT;
    public static double PATTERN_SEEK_TIMEOUT_S = 2.5;

    // Spindex decisions
    public static boolean PREFER_CLOCKWISE_ON_TIE = true;

    // Intake forward phase
    public static double INTAKE_FORWARD_SPEED = 0.25;
    public static double INTAKE_FORWARD_TIMEOUT_S = 2.0;

    // Shooter/deposit
    public static double TARGET_RPM_BAND = 100;
    public static double RPM_VERIFY_DROP = ShooterConfig.RPM_AT_SHOT;
    public static double AT_RPM_WAIT_TIMEOUT_S = 1.2;
    public static double INDEX_DWELL_S = 0.30;
    public static double VERIFY_WINDOW_S = 0.50;
    public static int REFIRE_MAX = 1;

    // Spindexer jiggle
    public static int JIGGLE_MAX = 2;
    public static double JIGGLE_DELTA_UP = 0.10;
    public static double JIGGLE_DELTA_DOWN = 0.20;
    public static double JIGGLE_DWELL_S = 0.12;

    // Distance thresholds (cm)
    public static int DIST_SMOOTH_WINDOW = 5;
    public static double DIST_MAX_VALID_CM = 200.0;
    public static double DIST_WALL_CLOSE_CM = 13.0;

    // Fallback heading if no tag seen (deg)
    public static double FALLBACK_HEADING_RED_DEG = 0.0;
    public static double FALLBACK_HEADING_BLUE_DEG = 180.0;
}