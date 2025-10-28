package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterYawConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double TPR_OUTPUT = ((((1+((double) 46 /17))) * (1+((double) 46 /11))) * 28);
    public static double TPR_MOTOR = 28;
}
