package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
public class DriveConfig {
    public static boolean TELEMETRY_ENABLED = true;

    public static double ROBOT_MASS = 6.8;

    // Deadband for joystick input
    public static double STICK_DB = 0.05;
    public static double ROT_DB = 0.08;

    public static String FRONT_RIGHT_NAME = "front_right_drive";
    public static String BACK_RIGHT_NAME = "back_right_drive";
    public static String FRONT_LEFT_NAME = "front_left_drive";
    public static String BACK_LEFT_NAME = "back_left_drive";

    public static final DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BACK_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction FRONT_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BACK_LEFT_DIR = DcMotorSimple.Direction.REVERSE;
}
