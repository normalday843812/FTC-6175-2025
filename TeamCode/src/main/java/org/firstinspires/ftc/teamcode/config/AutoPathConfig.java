package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Configurable control points for autonomous PedroPathing Bézier paths.
 *
 * These arrays allow you to explicitly shape the curve the robot follows when
 * driving to key poses. Each entry is {@code {x, y, headingDeg}} in Pedro
 * coordinates. Leave an array empty to fall back to the default midpoint
 * control point so existing behavior is unchanged.
 */
@Configurable
public class AutoPathConfig {
    /** Control points used when driving from the start to the shooting pose. */
    public static double[][] TO_SHOOT_CONTROL_POINTS = {};

    /** Control points used when driving from a pickup location back to intake. */
    public static double[][] TO_INTAKE_CONTROL_POINTS = {};

    /** Control points used for the final park move. */
    public static double[][] TO_FINAL_CONTROL_POINTS = {};

    /**
     * T-value within the path where heading interpolation should end.
     * Defaults to the previous 80% behavior.
     */
    public static double HEADING_INTERPOLATION_END_T = 0.8;
}
