package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class HoodServoConfig {
    public static final boolean TELEMETRY_ENABLED = true;

    // Motion limits
    public static final double MAX_POS = 1.0;
    public static final double MIN_POS = 0.0;
    public static final double START_POS = 0.50;
    public static final double MANUAL_DEADBAND = 0.05;
    public static final double MANUAL_RATE_PER_SEC = 0.75; // position units per second
    public static final double MANUAL_HOLD_SEC = 0.3;
}
