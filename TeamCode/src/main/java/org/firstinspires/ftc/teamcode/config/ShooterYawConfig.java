package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

@Configurable
public class ShooterYawConfig {

    // Encoder conversion
    public static double TICKS_PER_DEG = 2.373;

    // Position limits
    public static int MIN_TICKS = -213;  // ±90°
    public static int MAX_TICKS = 213;

    // PID gains
    public static double KP = 0.007;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KS = 0.02;
    public static double KF = 0.0;

    // Output limits
    public static double MAX_POWER = 0.6;

    private static final Pose GOAL_BLUE_PEDRO = new Pose(0, 144, 45, FTCCoordinates.INSTANCE);
    private static final Pose GOAL_RED_PEDRO = new Pose(144, 144, 0, FTCCoordinates.INSTANCE);

    public static double GOAL_BLUE_X = GOAL_BLUE_PEDRO.getX();
    public static double GOAL_BLUE_Y = GOAL_BLUE_PEDRO.getY();
    public static double GOAL_RED_X = GOAL_RED_PEDRO.getX();
    public static double GOAL_RED_Y = GOAL_RED_PEDRO.getY();

    // Anti-windup
    public static double INTEGRAL_ZONE_TICKS = 50;
    public static double INTEGRAL_MAX = 1000;

    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;
}