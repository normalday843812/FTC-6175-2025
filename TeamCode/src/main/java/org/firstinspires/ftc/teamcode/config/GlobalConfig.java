package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class GlobalConfig {
    public static boolean COMPETITION_MODE = false;
    public static boolean ENABLE_TELEMETRY = true;
    public static boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static double DEFAULT_MAX_POWER = 1.0;
    public static boolean ENABLE_VISION = true;
    public static boolean isFailFastOnMissingHardware() {
        return !COMPETITION_MODE;
    }
}