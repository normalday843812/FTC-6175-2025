package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterYawConfig {

    // Encoder conversion
    public static double TICKS_PER_DEG = 2.373;

    // Aim trim (degrees added after goal tracking, before conversion to ticks)
    public static double AIM_BIAS_DEG = 0.0;
    public static double AIM_BIAS_STEP_DEG = 5.0;
    public static double AIM_BIAS_MIN_DEG = -30.0;
    public static double AIM_BIAS_MAX_DEG = 30.0;

    // Position limits
    public static int MIN_TICKS = -213;  // ±90°
    public static int MAX_TICKS = 213;

    // PID gains
    public static double KP = 0.007;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KS = 0.02;
    public static double KF = 0.0;

    // Output limits
    public static double MAX_POWER = 0.6;

    // Anti-windup
    public static double INTEGRAL_ZONE_TICKS = 50;
    public static double INTEGRAL_MAX = 1000;

    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;
}
