package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ColorCal {
    public static final int H = 0;
    public static final int S = 1;
    public static final int V = 2;
    public static final int A = 3;

    public static float[] SENSOR_GAIN = {40f, 40f, 45f};

    // Ball color classification is done via non-overlapping HSVA intervals.
    // If a sample does not fit ANY interval, it is treated as NO BALL.
    //
    // Defaults preserve the old "V/A threshold" behavior by:
    // - splitting Hue into 2 non-overlapping halves (0..180, 180..360)
    // - gating both colors with the same per-sensor V/A minimums

    // RED interval (Hue split #1)
    public static double[] RED_H_MIN = {0, 0, 0};
    public static double[] RED_H_MAX = {180, 180, 180};
    public static double[] RED_S_MIN = {0.0, 0.0, 0.0};
    public static double[] RED_S_MAX = {1.0, 1.0, 1.0};
    public static double[] RED_V_MIN = {0.13, 0.15, 0.035};
    public static double[] RED_V_MAX = {1.0, 1.0, 1.0};
    public static double[] RED_A_MIN = {0.34, 0.40, 0.025};
    public static double[] RED_A_MAX = {1.0, 1.0, 1.0};

    // BLUE interval (Hue split #2)
    public static double[] BLUE_H_MIN = {180, 180, 180};
    public static double[] BLUE_H_MAX = {360, 360, 360};
    public static double[] BLUE_S_MIN = {0.0, 0.0, 0.0};
    public static double[] BLUE_S_MAX = {1.0, 1.0, 1.0};
    public static double[] BLUE_V_MIN = {0.13, 0.15, 0.035};
    public static double[] BLUE_V_MAX = {1.0, 1.0, 1.0};
    public static double[] BLUE_A_MIN = {0.34, 0.40, 0.025};
    public static double[] BLUE_A_MAX = {1.0, 1.0, 1.0};

    // Ball presence semantics:
    // - Sensors 0/1 are the "front" (slot-0 / shooter-feed) sensors.
    // - Sensor 2 is a separate intake sensor.
    public static int FRONT_SENSOR_COUNT = 2;
    public static int FRONT_REQUIRED_COUNT = 1;

    public static boolean TELEMETRY_ENABLED = true;
}
