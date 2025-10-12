package org.firstinspires.ftc.teamcode.localisation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LocalisationConstants {
    // Limelight
    public static final long LL_STALE_MS = 250;
    public static final double ZERO_EPS_POS_M = 1e-6;
    public static final double ZERO_EPS_YAW_RAD = 1e-6;
    public static final int MIN_TAGS = 1;
    public static final double SINGLE_TAG_3SIGMA_POS_MAX_M = 0.50;
    public static final double MIN_AVG_AREA = 0.0;

    // Dynamic gate clamps
    public static final double GATE_POS_MIN_M = 0.15;
    public static final double GATE_POS_MAX_M = 0.60;
    public static final double GATE_YAW_MIN_RAD = Math.toRadians(5);
    public static final double GATE_YAW_MAX_RAD = Math.toRadians(20);

    public static final double SNAP_POS_MAX_M = 0.60;
    public static final double SNAP_YAW_MAX_RAD = Math.toRadians(30);

    // Stationary
    public static final double V_STAT_MPS = 0.05;
    public static final double OMEGA_STAT_RADPS = 0.15;
    public static final int OMEGA_WINDOW = 5;
    public static final long STATIONARY_HOLD_MS = 200;
    public static final double OMEGA_CLAMP_RADPS = 8.0;

    // Other
    public static final boolean TELEMETRY_ENABLED = true;
    public static final double kPos = 0.25;
    public static final double kTheta = 0.5;
}
