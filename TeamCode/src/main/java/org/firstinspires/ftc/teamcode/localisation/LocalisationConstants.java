package org.firstinspires.ftc.teamcode.localisation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LocalisationConstants {
    // Limelight
    public static long LL_STALE_MS = 250;
    public static double ZERO_EPS_POS_M = 1e-6;
    public static double ZERO_EPS_YAW_RAD = 1e-6;
    public static int MIN_TAGS = 1;
    public static double SINGLE_TAG_3SIGMA_POS_MAX_M = 0.50;
    public static double MIN_AVG_AREA = 0.0;

    // Dynamic gate clamps
    public static double GATE_POS_MIN_M = 0.15;
    public static double GATE_POS_MAX_M = 0.60;
    public static double GATE_YAW_MIN_RAD = Math.toRadians(5);
    public static double GATE_YAW_MAX_RAD = Math.toRadians(20);

    public static double SNAP_POS_MAX_M = 0.60;
    public static double SNAP_YAW_MAX_RAD = Math.toRadians(30);

    // Stationary
    public static double V_STAT_MPS = 0.05;
    public static double OMEGA_STAT_RADPS = 0.15;
    public static int OMEGA_WINDOW = 5;
    public static long STATIONARY_HOLD_MS = 200;
    public static double OMEGA_CLAMP_RADPS = 8.0;

    // Pinpoint
    public static double PINPOINT_X_OFFSET_M = 0.229;
    public static double PINPOINT_Y_OFFSET_M = 0.127;
}
