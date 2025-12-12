package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ColorCal {
    public static final int H = 0;
    public static final int S = 1;
    public static final int V = 2;
    public static final int A = 3;

    public static float[] SENSOR_GAIN = {40f, 40f, 45f};
    public static double[] V_THRESHOLD = {
            0.10,  // Sensor 0
            0.15,  // Sensor 1
            0.035  // Sensor 2 (intake)
    };

    public static double[] A_THRESHOLD = {
            0.15,  // Sensor 0
            0.15,  // Sensor 1
            0.025  // Sensor 2 (intake)
    };

    public static boolean REQUIRE_BOTH_VA = false;

    public static boolean TELEMETRY_ENABLED = true;
}