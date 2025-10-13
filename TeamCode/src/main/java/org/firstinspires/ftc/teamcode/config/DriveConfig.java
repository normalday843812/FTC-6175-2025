package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Configurable
public class DriveConfig {
    public static boolean TELEMETRY_ENABLED = true;
    public static double SLOW_MODE_FACTOR = 0.5;
    public static double KP_YAW = 0.01;
    public static double KD_YAW = 0.0;
    public static double STICK_DB = 0.05;
    public static double ROT_DB = 0.08;
    public static double OMEGA_MAX = 1.0;

    public static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static double PINPOINT_X_OFFSET_M = 0.229;
    public static double PINPOINT_Y_OFFSET_M = 0.127;
}
