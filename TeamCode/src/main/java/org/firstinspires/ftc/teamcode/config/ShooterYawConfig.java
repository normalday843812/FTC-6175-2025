package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterYawConfig {

    // Encoder conversion
    public static double TICKS_PER_DEG = 2.373;

    // Position limits
    public static int CENTER_TICKS = 0;
    public static int MIN_TICKS = -213;  // ±90°
    public static int MAX_TICKS = 213;

    // PID gains
    public static double KP = 0.007;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KS = 0.02;       // Static friction
    public static double KF = 0.0;      // Feedforward for robot rotation (deg/sec to power)

    // Output limits
    public static double MAX_POWER = 0.6;

    // Vision
    public static long TAG_STALE_MS = 500;

    // Pattern seeking
    public static double SCAN_AMPLITUDE_DEG = 15.0;
    public static long SCAN_PERIOD_MS = 2000;
    public static long SEEK_TIMEOUT_MS = 3000;
    public static long STABLE_HOLD_MS_DEFAULT = 350L;

    // Anti-windup
    public static double INTEGRAL_ZONE_TICKS = 50;
    public static double INTEGRAL_MAX = 1000;

    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;
}