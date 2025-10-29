package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterYawConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double TPR_OUTPUT = ((((1+((double) 46 /17))) * (1+((double) 46 /11))) * 28);
    public static double TPR_MOTOR = 28;
    public static double YAW_POWER = 0.6;
    public static int MIN_POSITION = -100;
    public static int MAX_POSITION = 100;
    public static double CONTROL_LEVEL = 5;
}
