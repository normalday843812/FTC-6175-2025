package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;

@Configurable
public class ShooterYawConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Mechanism
    public static double YAW_POWER = 0.6;
    public static int MIN_POSITION = -340;
    public static int MAX_POSITION = 450;
    public static int CENTER_POS = 0;
    public static double CONTROL_LEVEL = 20;

    // Auto lock enable
    public static boolean AUTO_LOCK_ENABLED = true;

    // Pedro FilteredPIDF gains for yaw

    public static FilteredPIDFCoefficients PIDF_COEFFICIENTS = new FilteredPIDFCoefficients(
            0.035, 0.000, 0.005, 0.6, 0
    );





    /**
     * Derivative low-pass time constant for FilteredPIDFController (0..1 typical).
     */
    public static double AUTO_LOCK_T = 0.6;

    public static double AUTO_LOCK_MAX_POWER = 0.6;
    public static double AUTO_LOCK_DEADBAND_DEG = 1.5;

    public static int SOFT_LIMIT_MARGIN = 2;

    // ---------- Aim/search & fusion (kept here to avoid extra config files) ----------

    /**
     * Sinusoidal scan amplitude when seeking (±deg).
     */
    public static double SCAN_BAND_DEG = 12.0;
    /**
     * Scan period (ms).
     */
    public static long SCAN_PERIOD_MS = 1200;
    /**
     * Per-candidate seek window (ms) when pattern seeking.
     */
    public static long SEEK_WINDOW_MS = 1200;

    /**
     * Distance^2 coefficient for Limelight trust weighting. Larger → trust less at distance.
     */
    public static double LL_WEIGHT_DIST2 = 3.0;

    // Pedro KalmanFilter parameters on heading error (deg)
    public static double KF_Q_IMU = 0.05;     // process covariance for IMU-geometry error
    public static double KF_R_IMU = 0.25;     // measurement covariance for IMU-geometry error

    public static double KF_Q_LL = 0.10;     // process covariance for Limelight yaw error
    public static double KF_R_LL_BASE = 1.00; // base measurement covariance for Limelight yaw

    /**
     * Default hold time to "confirm" a pattern tag during autonomous (ms).
     */
    public static long STABLE_HOLD_MS_DEFAULT = 350;

    /**
     * Convert PedroPathing pose units to meters for tag metadata. If pose is inches, use 0.0254.
     */
    public static double METERS_PER_POSE_UNIT = 0.0254;
}
