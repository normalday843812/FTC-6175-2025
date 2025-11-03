package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LLAprilTagConfig {
    public static boolean TELEMETRY_ENABLED = true;

    public static long TTL_MS = 150;

    public static double MIN_TAG_AREA = 0.0;
    public static double MAX_TAG_DIST_M = 10.0;
}
