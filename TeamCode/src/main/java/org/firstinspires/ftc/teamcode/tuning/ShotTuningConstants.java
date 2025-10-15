package org.firstinspires.ftc.teamcode.tuning;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShotTuningConstants {
    // Limelight yaw lock on tx (radians)
    public static final double YAW_KP = 0.025;
    public static final double YAW_KD = 0.000;
    public static final double OMEGA_MAX = 1.2; // rad/s clamp

    // Driver scaling
    public static final double DRIVE_SCALE = 0.75;
    public static final double ROTATE_SCALE = 1.0;
    public static final double STICK_DB = 0.05;

    // Test grid
    public static final double[] RPM_LIST = {2600, 3000, 3400, 3800, 4200};
    public static final double[] HOOD_LIST = {0.20, 0.30, 0.40, 0.50, 0.60};

    // “Settled” gates (log only from rest)
    public static final double MAX_VEL_MPS = 0.05;
    public static final double MAX_OMEGA_RAD = 0.15;

    // Limelight
    public static final int LIMELIGHT_PIPELINE = 0;

    // Telemetry table depth
    public static final int TELEMETRY_ROWS = 8;
}
