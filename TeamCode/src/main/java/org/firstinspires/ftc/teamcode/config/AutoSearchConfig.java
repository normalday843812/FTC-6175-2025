package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoSearchConfig {
    // Search scan timing
    public static double ROTATE_SCAN_TIME_S = 2.0;
    public static double FORWARD_SCAN_TIME_S = 2.0;
    public static double SPIN_IN_PLACE_TIME_S = 3.0;
    public static double SEARCH_BUDGET_S = 12.0;

    // Rotating
    public static double ROTATE_SCAN_SPEED = 0.18;
    public static double SPIN_IN_PLACE_SPEED = 0.22;

    // Target centers for half of the field
    public static double RED_HALF_CENTER_X = 108.0;
    public static double RED_HALF_CENTER_Y = 72.0;

    public static double BLUE_HALF_CENTER_X = 36.0;
    public static double BLUE_HALF_CENTER_Y = 72.0;

    // Intake/approach
    public static double BLOB_APPROACH_GAIN = 0.6;
    public static double BLOB_FORWARD_MAX = 0.6;
    public static double BLOB_STRAFE_MAX = 0.5;
    // Keep intake after losing blob to catch rolling piece
    public static double LOST_BALL_EXTRA_INTAKE_S = 0.4;

    // Acceptance logic windows
    // After this time, accept any seen blob even if not needed
    public static double OVERRIDE_TIME_S = 8.0;
}