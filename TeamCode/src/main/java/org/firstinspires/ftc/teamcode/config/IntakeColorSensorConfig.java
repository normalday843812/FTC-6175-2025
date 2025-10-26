package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeColorSensorConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static float HUE_MIN_PURPLE = 205f;
    public static float HUE_MAX_PURPLE = 245f;
    public static float SATURATION_MIN_PURPLE = 0.30f;
    public static float VALUE_MIN_PURPLE = 0.15f;

    public static float HUE_MIN_GREEN = 75f;
    public static float HUE_MAX_GREEN = 150f;
    public static float SATURATION_MIN_GREEN = 0.30f;
    public static float VALUE_MIN_GREEN = 0.15f;
    public static float GAIN = 100f;

    public static final int WINDOW_SIZE = 7; // Window Size
    public static final int N = 4; // Required matches
}
