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
    // Hue is in degrees (0..360) because we use android.graphics.Color.RGBToHSV().
    //
    // Defaults preserve the old "V/A threshold" behavior by:
    // - using broad hue ranges for PURPLE/GREEN
    // - gating both colors with the same per-sensor V/A minimums

    // PURPLE interval
    public static double[] PURPLE_H_MIN = {185.0, 200.0, 180.0};
    public static double[] PURPLE_H_MAX = {240.0, 240.0, 240.0};
    public static double[] PURPLE_S_MIN = {0.3, 0.3, 0.3};
    public static double[] PURPLE_S_MAX = {0.49, 0.49, 0.49};
    public static double[] PURPLE_V_MIN = {0.02, 0.02, 0.02};
    public static double[] PURPLE_V_MAX = {1.0, 1.0, 0.6};
    public static double[] PURPLE_A_MIN = {0.065, 0.1, 0.03};
    public static double[] PURPLE_A_MAX = {0.91, 0.91, 0.3};

    // GREEN interval
    public static double[] GREEN_H_MIN = {140.0, 140.0, 150.0};
    public static double[] GREEN_H_MAX = {180, 180, 180};
    public static double[] GREEN_S_MIN = {0.5, 0.5, 0.5};
    public static double[] GREEN_S_MAX = {0.8, 0.8, 0.75};
    public static double[] GREEN_V_MIN = {0.03, 0.03, 0.03};
    public static double[] GREEN_V_MAX = {0.5, 0.5, 0.1};
    public static double[] GREEN_A_MIN = {0.04, 0.04, 0.018};
    public static double[] GREEN_A_MAX = {0.8, 0.8, 0.1};

    // Ball presence semantics:
    // - Sensors 0/1 are the "front" (slot-0 / shooter-feed) sensors.
    // - Sensor 2 is a separate intake sensor.
    public static int FRONT_SENSOR_COUNT = 2;
    public static int FRONT_REQUIRED_COUNT = 1;

    public static boolean TELEMETRY_ENABLED = true;
}
