package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoUnifiedConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Timeouts
    public static double DEFAULT_TIMEOUT_S = 2.5;
    public static double PATH_TIMEOUT_TO_GOAL_S = 3.0;
    public static double PATH_TIMEOUT_TO_INTAKE_S = 3.0;
    public static double ROTATE_NEXT_BALL_TIMEOUT_S = 3.0;
    public static double ROTATE_NEXT_BALL_SENSOR_SETTLE_S = 0.20;

    // Spindex decisions
    public static boolean PREFER_CLOCKWISE_ON_TIE = true;

    // Intake creep phase
    public static double INTAKE_CREEP_DISTANCE = 5.0;  // X offset in Pedro coordinates (increased for better intake)
    public static double INTAKE_FORWARD_TIMEOUT_S = 5.0;  // Longer timeout to actually intake balls
    public static int INTAKE_CONFIRM_CYCLES = 3;  // consecutive reads after empty->ball transition
    public static double INTAKE_POST_DETECT_DWELL_S = 0.15; // extra settle time after detecting a new ball
    public static boolean AUTO_IDLE_INTAKE_FORWARD = true; // run intake FORWARD except during INTAKING
    public static boolean AUTO_IDLE_INTAKE_FORWARD_DURING_SHOOTING = true;
    public static boolean AUTO_IDLE_INTAKE_FORWARD_DURING_ROTATE = true;
    public static boolean REQUIRE_FULL_BEFORE_SHOOT = true; // only shoot when 3 balls unless out of sets/time

    // Inventory audit (slot-0 sensor reused by rotating spindexer)
    public static boolean INVENTORY_AUDIT_ENABLED = true;
    public static double INVENTORY_AUDIT_SENSOR_SETTLE_S = 0.3;
    public static int INVENTORY_AUDIT_CONFIRM_CYCLES = 3;
    public static double INVENTORY_AUDIT_TIMEOUT_S = 3.0;

    // Front-clear recovery when we can't align an empty slot (slot-0 sensor stuck reading BALL)
    public static boolean FRONT_CLEAR_ENABLED = true;
    public static int FRONT_CLEAR_MAX_ATTEMPTS = 3;
    public static double FRONT_CLEAR_REVERSE_DWELL_S = 0.25;
    public static double FRONT_CLEAR_FORWARD_DWELL_S = 0.10;
    public static int FRONT_CLEAR_EMPTY_CONFIRM_CYCLES = 2;

    // Rotate-to-ball recovery (when we expect a ball, but the front sensors read empty)
    public static boolean ROTATE_RESEAT_ENABLED = true;
    public static int ROTATE_RESEAT_MAX_ATTEMPTS = 2;
    public static double ROTATE_RESEAT_REVERSE_DWELL_S = 0.25;
    public static double ROTATE_RESEAT_SETTLE_DWELL_S = 0.10;

    // Transfer holding behavior (lever is still raised for retention)
    public static boolean AUTO_HOLD_TRANSFER_DURING_SHOOTING = true;
    public static boolean AUTO_HOLD_TRANSFER_DURING_ROTATE = true;

    // Shooting behavior
    // If we enter SHOOTING but slot-0 sensors say empty, immediately rotate rather than waiting for DepositController to time out.
    public static double AUTO_SHOOT_EMPTY_PRECHECK_S = 0.25;
    // Keep the shooter at AUTO_TARGET_RPM while rotating between balls so we don't "spin down then wait to spin up"
    // between shots (often perceived as long 'spindexing' delays at the goal).
    public static boolean AUTO_KEEP_SHOOTER_SPUN_UP_BETWEEN_SHOTS = true;

    // Shooter/deposit
    public static double AUTO_TARGET_RPM = 3500;
    public static double TARGET_RPM_BAND = 500;
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
    public static double TELEOP_FRONT_SEAT_DWELL_S = 0.15; // wait after slot-0 sensors first see ball before spindexing

    // Teleop manager behavior
    public static boolean TELEOP_MANAGER_TELEMETRY_ENABLED = true;
    public static double TELEOP_SHOT_TIMEOUT_S = 1.0;
    public static int TELEOP_INTAKE_CONFIRM_CYCLES = 3;
    public static double TELEOP_LOAD_TIMEOUT_S = 1.2;
    public static int TELEOP_LOAD_FRONT_CONFIRM_CYCLES = 2;

    // Teleop spindex jam recovery (sensor verification uses slot-0/front detection)
    public static double TELEOP_SPINDEX_VERIFY_DELAY_S = 0.15;
    public static double TELEOP_SPINDEX_JIGGLE_DELTA = 0.06;
    public static double TELEOP_SPINDEX_JIGGLE_DWELL_S = 0.12;
    public static int TELEOP_SPINDEX_MAX_RECOVERY_ATTEMPTS = 1;
}
