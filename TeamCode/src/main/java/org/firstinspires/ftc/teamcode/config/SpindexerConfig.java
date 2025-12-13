package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class SpindexerConfig {
    public static boolean TELEMETRY_ENABLED = true;

    public static double STEP = 0.383;
    public static double BIAS = 0;
    public static int SLOTS = 3;

    // Time-based "settled" detection for the spindexer servo.
    // The servo has no positional feedback, so we treat it as moving for this long after a command.
    public static long SETTLE_TIME_MS = 300;
}
