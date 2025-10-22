package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class ShooterConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double TPR_OUTPUT = 384.5397923875;
    public static double TPR_MOTOR = 28.0;
    public static double P = 10.00;
    public static double I = 3.00;
    public static double D = 0.00;
    public static double F = 0.000542;
    public static double TRIGGER_SCALE = 20;
    public static double MAX_RPM = 5000;
    public static DcMotorSimple.Direction shooterMotorDirection = DcMotorSimple.Direction.FORWARD;
}
