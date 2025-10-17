package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class HoodServoConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Motion limits
    public static double MAX_POS = 1.0;
    public static double MIN_POS = 0.0;
    public static double START_POS = 0.50; // Directly after initialisation
    public static double MANUAL_DEADBAND = 0.05;
    public static double MANUAL_RATE_PER_SEC = 0.75; // position units per second
    public static double MANUAL_HOLD_SEC = 0.3;
}
