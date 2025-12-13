package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleOpShooterConfig {
    public static double IDLE_HEADING_DEG = 270.0;
    public static double IDLE_HEADING_TOLERANCE_DEG = 20.0;

    // Manual RPM override (primarily for TeleopManager disabled/manual mode, and for "full-load" shooting)
    public static boolean MANUAL_RPM_OVERRIDE_ENABLED = true;
    public static double MANUAL_RPM_OVERRIDE_TRIGGER_DEADBAND = 0.05;
    public static double MANUAL_RPM_OVERRIDE_RATE_RPM_PER_S = 3500.0;
    public static double MANUAL_RPM_OVERRIDE_MIN_RPM = 800.0;
    public static double MANUAL_RPM_OVERRIDE_MAX_RPM = 3800.0;

    // When TeleopManager is enabled, only allow starting an override when we are FULL (3 balls).
    // Once latched, it stays active until we become empty (or ClearAll is pressed).
    public static boolean MANUAL_RPM_OVERRIDE_LATCH_REQUIRES_FULL = true;
}
