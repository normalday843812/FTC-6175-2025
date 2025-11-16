package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeColorSensorConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static float HUE_MIN_PURPLE = 200f;
    public static float HUE_MAX_PURPLE = 255f;
    public static float SATURATION_MIN_PURPLE = 0.32f;
    public static float VALUE_MIN_PURPLE = 0.12f;

    public static float HUE_MIN_GREEN = 150f;
    public static float HUE_MAX_GREEN = 185f;
    public static float SATURATION_MIN_GREEN = 0.55f;
    public static float VALUE_MIN_GREEN = 0.08f;

    public static float ALPHA_MIN_BALL = 20f;
    public static float ALPHA_MIN_PURPLE = 45f;
    public static float ALPHA_MIN_GREEN = 22f;

    public static float GAIN = 45f;

    public static final int WINDOW_SIZE = 7; // Window Size
    public static final int N = 4; // Required matches
}
