package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleOpShooterConfig {
    // Manual RPM override (primarily for TeleopManager disabled/manual mode, and for "full-load" shooting)
    public static boolean MANUAL_RPM_OVERRIDE_ENABLED = true;
    public static double MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND = 0.05;
    public static double MANUAL_RPM_OVERRIDE_RATE_RPM_PER_S = 3500.0;
    public static double MANUAL_RPM_OVERRIDE_MIN_RPM = 800.0;
    public static double MANUAL_RPM_OVERRIDE_MAX_RPM = 3800.0;

    // When TeleopManager is enabled, only allow starting an override when we are FULL (3 balls).
    // Once latched, it stays active until we become empty (or ClearAll is pressed).
    public static boolean MANUAL_RPM_OVERRIDE_LATCH_REQUIRES_FULL = true;

    // Optional: distance-based shooter RPM model (teleop only).
    // Uses Limelight AprilTag distance to the *alliance* goal tag (Blue=20, Red=24).
    // If the tag is not fresh/visible, falls back to AutoUnifiedConfig.AUTO_TARGET_RPM.
    public static boolean TELEOP_DISTANCE_RPM_MODEL_ENABLED = false;
    // Distances are in inches (Limelight reports meters; code converts).
    public static double TELEOP_DISTANCE_RPM_MIN_DIST_IN = 39.37;
    public static double TELEOP_DISTANCE_RPM_AT_MIN_DIST = 3000.0;
    public static double TELEOP_DISTANCE_RPM_MAX_DIST_IN = 157.48;
    public static double TELEOP_DISTANCE_RPM_AT_MAX_DIST = 3800.0;
}
