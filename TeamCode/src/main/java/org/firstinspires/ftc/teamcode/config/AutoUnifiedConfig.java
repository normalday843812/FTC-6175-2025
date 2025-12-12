package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoUnifiedConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Timeouts
    public static double DEFAULT_TIMEOUT_S = 2.5;
    public static double PATH_TIMEOUT_TO_GOAL_S = 3.0;
    public static double PATH_TIMEOUT_TO_INTAKE_S = 3.0;

    // Spindex decisions
    public static boolean PREFER_CLOCKWISE_ON_TIE = true;

    // Intake forward phase
    public static double INTAKE_FORWARD_SPEED = 0.35;
    public static double INTAKE_FORWARD_TIMEOUT_S = 1.2;

    // Shooter/deposit
    public static double AUTO_TARGET_RPM = ShooterConfig.MAX_RPM;
    public static double TARGET_RPM_BAND = 150;
    public static double RPM_VERIFY_DROP = ShooterConfig.SHOT_DROP_RPM;
    public static double AT_RPM_WAIT_TIMEOUT_S = 0.6;
    public static double INDEX_DWELL_S = 0.10;
    public static double VERIFY_WINDOW_S = 0.25;
    public static int REFIRE_MAX = 1;

    // Spindexer jiggle
    public static int JIGGLE_MAX = 1;
    public static double JIGGLE_DELTA_UP = 0.10;
    public static double JIGGLE_DELTA_DOWN = 0.20;
    public static double JIGGLE_DWELL_S = 0.08;

    // Teleop manager timings
    public static double TELEOP_FEED_DWELL_S = 1.00;
}