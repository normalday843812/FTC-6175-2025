package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleopSortManagerConfig {
    // delay before spindexing after ball detected (gives transfer time to grip ball)
    public static double BALL_GRIP_DELAY_S = 2.0;
    
    // how long transfer stays raised after spindexing completes
    public static double TRANSFER_LOWER_DELAY_S = 1.0;
    
    // cooldown after manual spindex control before auto-spindex can trigger
    public static double MANUAL_SPINDEX_COOLDOWN_S = 1.0;
}
