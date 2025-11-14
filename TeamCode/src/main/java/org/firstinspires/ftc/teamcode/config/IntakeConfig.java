package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double FORWARD_PWR = -1.0;
    public static double REVERSE_PWR = 1.0;

    public static double JAM_OUT_TIME_S = 0.1;
    public static double JAM_IN_TIME_S = 0.1;

    public static double INTAKE_TPR = 28.0;
    public static double JAM_FWD_MIN_RPM = 50.0;
    public static double JAM_REV_MIN_ABS_RPM = 50.0;
    public static final double FWD_ARM_FACTOR = 3.0;
    public static final double REV_ARM_FACTOR = 3.0;
}
