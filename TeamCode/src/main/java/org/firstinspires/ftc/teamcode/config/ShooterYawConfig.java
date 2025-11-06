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

    public static double TICKS_PER_DEG_INIT = 3.0;
    public static double AUTO_LOCK_T = 0.6;

    public static double AUTO_LOCK_MAX_POWER = 0.8;
    public static double AUTO_LOCK_DEADBAND_DEG = 1.5;

    public static boolean IMU_HOLD_ENABLED = true;
    public static double KP_IMU = 1.0;
    public static double MAX_SLEW_TICKS_PER_SEC = 500.0;

    public static FilteredPIDFCoefficients PIDF_COEFFICIENTS = new FilteredPIDFCoefficients(0.8,0 ,0.7, 0.80, 0);

    public static int SOFT_LIMIT_MARGIN = 2;
    public static double SCAN_BAND_DEG = 12.0;
    public static long SCAN_PERIOD_MS = 1200;
    public static long SEEK_WINDOW_MS = 1200;

    public static double LL_WEIGHT_DIST2 = 3.0;

    public static double KF_Q_IMU = 0.05;
    public static double KF_R_IMU = 0.25;
    public static double KF_Q_LL = 0.10;
    public static double KF_R_LL_BASE = 1.00;
    public static long STABLE_HOLD_MS_DEFAULT = 350;
    public static double METERS_PER_POSE_UNIT = 0.0254;
}
