package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class TransferConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double TRANSFER_1_MIN = 0.38;
    public static double TRANSFER_1_MIN_SHOOTING = 0.24;
    public static double TRANSFER_1_MAX = 0.1;

    public static double FLICK_TIME_S = 0.12;
    public static double RESET_TIME_S = 0.1;

    public static Servo.Direction SERVO_1_DIRECTION = Servo.Direction.FORWARD;
    public static DcMotorSimple.Direction SERVO_2_DIRECTION = DcMotorSimple.Direction.REVERSE;
}
