package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@Configurable
public class LocalisationConfig {
    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    // Pinpoint configuration
    public static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection FORWARD_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection STRAFE_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static double PINPOINT_X_OFFSET_IN = 7;
    public static double PINPOINT_Y_OFFSET_IN = -1.5;
}
