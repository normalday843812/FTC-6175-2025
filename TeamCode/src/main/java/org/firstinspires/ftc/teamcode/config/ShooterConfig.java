package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class ShooterConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static boolean MOTOR1_ENABLED = true;
    public static double TPR_MOTOR = 28.0;
    public static double P = 10.00;
    public static double I = 3.00;
    public static double D = 0.00;
    public static double F = 0.000542;
    public static double IDLE_RPM = 800.0;
    public static double MAX_RPM = 3800;
    public static DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION_1 = DcMotorSimple.Direction.REVERSE;
    public static double SHOT_ARM_AT_RPM = 3500;
    public static long SHOT_ARM_DWELL_MS = 200;
    public static double SHOT_DROP_RPM = 500.0;
    public static long SHOT_DROP_WINDOW_MS = 350;
}
