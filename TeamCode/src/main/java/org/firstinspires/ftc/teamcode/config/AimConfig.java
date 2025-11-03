package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AimConfig {
    public static boolean AIM_TELEMETRY_ENABLED = true;

    public static double SCAN_BAND_DEG = 12.0;
    public static long SCAN_PERIOD_MS = 1200;
    public static long SEEK_WINDOW_MS = 1200;

    public static double LL_WEIGHT_DIST2 = 3.0;

    public static double KF_Q_IMU = 0.05;
    public static double KF_R_IMU = 0.25;

    public static double KF_Q_LL = 0.10;
    public static double KF_R_LL_BASE = 1.00;
}
