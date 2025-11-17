package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RgbIndicatorConfig {
    public static double RED_POS = 0.333; // Supposed to be 0.277
    public static double GREEN_POS = 0.500;
    public static double OFF_POS = 0.0;
    public static final double[] hueBreakpointsDeg = {0, 30, 60, 90, 120, 180, 240, 270, 300};
    public static final double[] positionBreakpoints =
            {0.277, 0.333, 0.388, 0.444, 0.500, 0.555, 0.611, 0.666, 0.722};

}
