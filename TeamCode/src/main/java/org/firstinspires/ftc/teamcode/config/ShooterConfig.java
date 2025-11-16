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
    public static double TRIGGER_SCALE_DOWN = 50;
    public static double TRIGGER_SCALE_UP = 50;
    public static double IDLE_RPM = 0;
    public static double MAX_RPM = 3500;
    public static double RPM_AT_SHOT = 2800;
    public static DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction SHOOTER_MOTOR_DIRECTION_1 = DcMotorSimple.Direction.REVERSE;
    public static boolean MOTOR_1_ENABLED = true;
}
