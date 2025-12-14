package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LLAprilTagConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Limelight settings (Teleop relocalization uses these)
    public static boolean ENABLED = true;
    public static int POLL_RATE_HZ = 100;
    public static int PIPELINE = 0;

    public static long TTL_MS = 100;

    public static double MIN_TAG_AREA = 0.0;
    public static double MAX_TAG_DIST_M = 10.0;
    public static final double YAW_FILTER_ALPHA = 0.2;
}
