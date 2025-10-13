package org.firstinspires.ftc.teamcode.localisation;

public class LocalisationConstants {
    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    // Units
    public static final double M_TO_IN = 39.37007874015748;
    public static final double IN_TO_M = 1.0 / M_TO_IN;

    // Limelight
    public static long LL_STALE_MS = 250;
    public static double ZERO_EPS_POS_M = 1e-6;
    public static double ZERO_EPS_YAW_RAD = 1e-6;
    public static double MIN_AVG_AREA = 0.0;
    public static double SINGLE_TAG_3SIGMA_POS_MAX_M = 0.50;

    // Dynamic gate clamps
    public static double GATE_POS_MIN_M = 0.15;
    public static double GATE_POS_MAX_M = 0.60;
    public static double GATE_YAW_MIN_RAD = Math.toRadians(5);
    public static double GATE_YAW_MAX_RAD = Math.toRadians(20);

    private LocalisationConstants() {}
}