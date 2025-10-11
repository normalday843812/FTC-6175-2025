package org.firstinspires.ftc.teamcode.localisation;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static boolean TELEMETRY_ENABLED = true;
    public static double OUTLIER_POS_M = 3.0; // TODO
    public static double TICKS_PER_REV = 2000.0;
    public static double WHEEL_RADIUS_M = 0.016;
    public static double GEAR_RATIO = 1.0;

    // Odometry wheel offsets relative to the robot center (meters):
    // +X is forward, +Y is robot-left.
    public static double PARALLEL_Y_M = 0.0; // lateral offset of the parallel (forward) wheel
    public static double PERPENDICULAR_X_M = 0.0; // forward offset of the perpendicular (strafe) wheel

    public static double kPos = 0.25;

    public static double kTheta = 0.5;

    public static final double GATE_POS_M = 2.0;
    public static final double GATE_TH_RAD = Math.toRadians(45);
}
