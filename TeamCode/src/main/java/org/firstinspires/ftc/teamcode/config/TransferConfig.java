package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class TransferConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double TRANSFER_1_MIN = 0.52;
    public static double TRANSFER_1_LEVER_UP = 0.7;
    public static double TRANSFER_1_MAX = 0.9;

    public static double FLICK_TIME_S = 1.0;
    public static double RESET_TIME_S = 0.05;

    public static Servo.Direction SERVO_1_DIRECTION = Servo.Direction.REVERSE;
    public static DcMotorSimple.Direction SERVO_2_DIRECTION = DcMotorSimple.Direction.REVERSE;
}
