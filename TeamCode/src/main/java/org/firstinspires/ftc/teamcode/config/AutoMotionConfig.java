package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoMotionConfig {
    // TODO: tune all of this
    // Heading PD
    public static double HEADING_KP = 1.0;
    public static double HEADING_KD = 0.02;
    public static double HEADING_MAX_ROT = 0.7;
    public static double HEADING_DEADBAND_DEG = 0.5;

    // Field translation
    public static double DRIVE_APPROACH_GAIN = 0.06;
    public static double DRIVE_MAX_VEL = 0.95; // TODO: Update in future?
    public static double DRIVE_STOP_DIST_IN = 1.25;

    // General timeouts
    public static double DEFAULT_TIMEOUT_S = 4.0;
}
