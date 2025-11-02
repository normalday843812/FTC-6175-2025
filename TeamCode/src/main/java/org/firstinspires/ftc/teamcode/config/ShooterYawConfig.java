package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterYawConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Mechanism
    public static double YAW_POWER = 0.6;
    public static int MIN_POSITION = -340;
    public static int MAX_POSITION = 450;
    public static int CENTER_POS = 0;
    public static double CONTROL_LEVEL = 20;

    public static boolean AUTO_LOCK_ENABLED = true;

    public static double AUTO_LOCK_KP = 0.035;
    public static double AUTO_LOCK_KI = 0.000;
    public static double AUTO_LOCK_KD = 0.005;
    public static double AUTO_LOCK_T = 0.6;
    public static double AUTO_LOCK_MAX_POWER = 0.6;
    public static double AUTO_LOCK_DEADBAND_DEG = 1.5;

    public static int SOFT_LIMIT_MARGIN = 2;
}
