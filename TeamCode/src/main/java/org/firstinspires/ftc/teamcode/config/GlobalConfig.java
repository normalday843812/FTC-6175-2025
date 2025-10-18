package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class GlobalConfig {
    public static boolean ENABLE_TELEMETRY = true;
    public static boolean FALLBACK_MODE = false;
    public static boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static double DEFAULT_MAX_POWER = 1.0;
    public static boolean ENABLE_VISION = true;

    public static boolean FAIL_FAST_ON_MISSING_HARDWARE = false;

    public static boolean isFailFastOnMissingHardware() {
        return FAIL_FAST_ON_MISSING_HARDWARE;
    }

    // Units
    public static final double M_TO_IN = 39.37007874015748;
    public static final double IN_TO_M = 1.0 / M_TO_IN;
}