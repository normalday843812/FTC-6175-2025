package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class DriveConfig {
    public static boolean TELEMETRY_ENABLED = true;

    // Velocity is multiplied by this amount
    public static double SLOW_MODE_FACTOR = 0.5;

    // PD controller for yaw
    public static double KP_YAW = 0.01;
    public static double KD_YAW = 0.0;

    // Deadband for joystick input
    public static double STICK_DB = 0.05;
    public static double ROT_DB = 0.08;

    // Maximum angular velocity (omega)
    public static double OMEGA_MAX = 1.0;
}
