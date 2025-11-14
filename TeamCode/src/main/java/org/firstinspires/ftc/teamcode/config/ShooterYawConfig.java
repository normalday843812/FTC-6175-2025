package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;

@Configurable
public class ShooterYawConfig {
    public static boolean TELEMETRY_ENABLED = true;

    public static int MIN_POSITION = -340;
    public static int MAX_POSITION = 450;
    public static int CENTER_POS = 0;
    public static double TICKS_PER_DEG_INIT = 3.0;

    public static double CONTROL_LEVEL = 20;

    public static double AUTO_LOCK_DEADBAND_DEG = 1.5;

    // RUN_WITHOUT_ENCODER
    public static double YAW_MAX_POWER = 1.0; // power cap while tuning
    public static double YAW_KS = 0.02; // static friction compensation

    public static long LL_GRACE_MS = 150;
    public static long REACQUIRE_AFTER_MS = 300;

    // Skip LL world-target updates when robot spinning very fast
    public static double OMEGA_LL_UPDATE_MAX_RAD = 3.0; // ~172 deg/s

    // Blend factor for world target updates
    public static double TARGET_BLEND_ALPHA = 0.6;

    // Scan/search
    public static boolean REACQUIRE_SCAN_ENABLED = true;
    public static int MIN_SCAN_AMPLITUDE_TICKS = 4;
    public static double SCAN_BAND_DEG = 12.0;
    public static long SCAN_PERIOD_MS = 1200;
    public static long SEEK_WINDOW_MS = 1200;
    public static long STABLE_HOLD_MS_DEFAULT = 350;

    public static int SOFT_LIMIT_MARGIN = 2;

    public static double METERS_PER_POSE_UNIT = 0.0254;

    public static double MAX_SLEW_TICKS_PER_SEC = 3000.0;

    public static FilteredPIDFCoefficients PIDF_COEFFICIENTS =
            new FilteredPIDFCoefficients(
                    0.012,
                    0.0,
                    0.0,
                    0.20,
                    0.0025
            );

    public static double KD_MEAS = 0.004;
    public static double VEL_LP_ALPHA = 0.2;

    public static double I_GAIN = 0.0;
    public static double I_MAX_POWER = 0.15;
}