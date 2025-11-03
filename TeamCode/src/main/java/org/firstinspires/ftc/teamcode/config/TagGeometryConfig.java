package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Unit conversion for bearing calc. If PedroPathing pose is inches, use 0.0254.
 */
@Configurable
public class TagGeometryConfig {
    public static double METERS_PER_POSE_UNIT = 0.0254;
}
