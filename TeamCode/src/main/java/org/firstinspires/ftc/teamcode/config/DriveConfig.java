package org.firstinspires.ftc.teamcode.config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Configurable
public class DriveConfig {
    public static final boolean TELEMETRY_ENABLED = true;
    public static final double SLOW_MODE_FACTOR = 0.5;
    public static final double KP_YAW = 0.01;
    public static final double KD_YAW = 0.0;
    public static final double STICK_DB = 0.05;
    public static final double ROT_DB = 0.08;
    public static final double OMEGA_MAX = 1.0;

    public static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final double PINPOINT_X_OFFSET_M = 0.229;
    public static final double PINPOINT_Y_OFFSET_M = 0.127;
}
