package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterYawConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Mechanism
    public static double TPR_OUTPUT = ((((1 + ((double) 46 / 17)) * (1 + ((double) 46 / 11))) * 28));
    public static double TPR_MOTOR = 28;
    public static double YAW_POWER = 0.6;
    public static int MIN_POSITION = -100;
    public static int MAX_POSITION = 100;
    public static int CENTER_POS = 0;
    public static double CONTROL_LEVEL = 5;

    public static boolean AUTO_LOCK_ENABLED = true;

    public static long APRILTAG_TTL_MS = 150;
    public static double MIN_TAG_AREA = 0.0;
    public static double MAX_TAG_DIST_M = 10.0;

    public static double AUTO_LOCK_KP = 0.035;
    public static double AUTO_LOCK_KI = 0.000;
    public static double AUTO_LOCK_KD = 0.005;
    public static double AUTO_LOCK_MAX_POWER = 0.6;
    public static double AUTO_LOCK_DEADBAND_DEG = 1.5;

    public static int SMOOTH_WINDOW = 3;          // moving average window (samples)

    // Soft limits (applied in AUTO_LOCK only)
    public static int SOFT_LIMIT_MARGIN = 2;      // ticks trimmed off each end
}
