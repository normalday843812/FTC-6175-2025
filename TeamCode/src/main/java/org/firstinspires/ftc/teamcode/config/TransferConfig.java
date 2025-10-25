package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TransferConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double MIN = 0.76;
    public static double MAX = 1.00;

    public static double FLICK_TIME_S = 0.1;
    public static double RESET_TIME_S = 0.1;
}
