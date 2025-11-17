package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class GlobalConfig {
    public static boolean ENABLE_TELEMETRY = true;
    public static boolean FALLBACK_MODE = false;

    public static boolean FAIL_FAST_ON_MISSING_HARDWARE = false;

    public static double SLOW_MODE_MULTIPLIER = 0.2;

    public static boolean isFailFastOnMissingHardware() {
        return FAIL_FAST_ON_MISSING_HARDWARE;
    }
}
