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

    // Intake creep phase
    public static double INTAKE_CREEP_DISTANCE = 5.0;  // X offset in Pedro coordinates (increased for better intake)
    public static double INTAKE_FORWARD_TIMEOUT_S = 5.0;  // Longer timeout to actually intake balls
    public static int INTAKE_CONFIRM_CYCLES = 3;  // consecutive reads after empty->ball transition
    public static double INTAKE_POST_DETECT_DWELL_S = 0.15; // extra settle time after detecting a new ball
    public static boolean AUTO_IDLE_INTAKE_FORWARD = true; // run intake FORWARD except during INTAKING
    public static boolean REQUIRE_FULL_BEFORE_SHOOT = true; // only shoot when 3 balls unless out of sets/time

    // Inventory audit (slot-0 sensor reused by rotating spindexer)
    public static boolean INVENTORY_AUDIT_ENABLED = true;
    public static double INVENTORY_AUDIT_SENSOR_SETTLE_S = 0.3;
    public static int INVENTORY_AUDIT_CONFIRM_CYCLES = 3;

    // Front-clear recovery when we can't align an empty slot (slot-0 sensor stuck reading BALL)
    public static boolean FRONT_CLEAR_ENABLED = true;
    public static int FRONT_CLEAR_MAX_ATTEMPTS = 3;
    public static double FRONT_CLEAR_REVERSE_DWELL_S = 0.25;
    public static double FRONT_CLEAR_FORWARD_DWELL_S = 0.10;
    public static int FRONT_CLEAR_EMPTY_CONFIRM_CYCLES = 2;

    // Shooter/deposit
    public static double AUTO_TARGET_RPM = 4000;
    public static double TARGET_RPM_BAND = 150;
    public static double AT_RPM_WAIT_TIMEOUT_S = 1.5;
    public static double INDEX_DWELL_S = 0.15;
    public static double VERIFY_WINDOW_S = 0.5;
    public static int REFIRE_MAX = 3;
    public static int DEPOSIT_EMPTY_CONFIRM_CYCLES = 2;  // consecutive empty reads after a flick

    // Spindexer jiggle
    public static int JIGGLE_MAX = 2;
    public static double JIGGLE_DELTA_UP = 0.10;
    public static double JIGGLE_DELTA_DOWN = 0.20;
    public static double JIGGLE_DWELL_S = 0.08;

    // Teleop manager timings
    public static double TELEOP_FEED_DWELL_S = 0.4;  // Time to push ball into bucket before indexing
}
