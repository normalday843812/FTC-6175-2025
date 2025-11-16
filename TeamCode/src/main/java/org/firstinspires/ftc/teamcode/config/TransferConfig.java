package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class TransferConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double TRANSFER_1_MIN = 0.96;
    public static double TRANSFER_1_MIN_SHOOTING = 0.8;
    public static double TRANSFER_1_MAX = 0.6;

    public static double FLICK_TIME_S = 0.7;
    public static double RESET_TIME_S = 0.05;
    public static double SHOOTING_MODE_DURATION_S = 0.5;

    public static Servo.Direction SERVO_1_DIRECTION = Servo.Direction.FORWARD;
    public static DcMotorSimple.Direction SERVO_2_DIRECTION = DcMotorSimple.Direction.REVERSE;
    
    // shooting mode cr servo direction: 1.0 for forward, -1.0 for reverse
    public static double SHOOTING_MODE_CR_POWER = -1.0;
}
