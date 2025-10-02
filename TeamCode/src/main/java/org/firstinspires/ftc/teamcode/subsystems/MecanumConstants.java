package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class MecanumConstants {
    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    // Motor constants
    public static double SLOW_MODE_FACTOR = 0.5; // TODO

    // PD
    public static double KP = 0.01; // TODO
    public static double KD = 0; // TODO

    // Deadbands & clamps
    public static double STICK_DEAD_BAND = 0.05; // TODO
    public static double ROTATE_DEAD_BAND = 0.08; // TODO
    public static double OMEGA_MAX = 1.0; // TODO
}
