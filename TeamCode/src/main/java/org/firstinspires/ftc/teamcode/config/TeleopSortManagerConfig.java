package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TeleopSortManagerConfig {
    // delay before spindexing after ball detected (gives transfer time to grip ball)
    public static double BALL_GRIP_DELAY_S = 2.0;
    
    // how long transfer stays raised after spindexing completes
    public static double TRANSFER_LOWER_DELAY_S = 1.0;
}
