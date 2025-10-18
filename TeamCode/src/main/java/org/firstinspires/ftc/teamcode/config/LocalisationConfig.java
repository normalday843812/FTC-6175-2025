package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Configurable
public class LocalisationConfig {
    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    // Limelight
    public static long LL_STALE_MS = 250;
    public static double ZERO_EPS_POS_M = 1e-6;
    public static double ZERO_EPS_YAW_RAD = 1e-6;
    public static double MIN_AVG_AREA = 0.0;

    // Dynamic gate clamps
    public static double GATE_POS_MIN_M = 0;
    public static double GATE_POS_MAX_M = 3.0;
    public static double GATE_YAW_MIN_RAD = Math.toRadians(5);
    public static double GATE_YAW_MAX_RAD = Math.toRadians(180);

    // Pinpoint configuration
    public static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static double PINPOINT_X_OFFSET_M = 0.229;
    public static double PINPOINT_Y_OFFSET_M = 0.127;
}
