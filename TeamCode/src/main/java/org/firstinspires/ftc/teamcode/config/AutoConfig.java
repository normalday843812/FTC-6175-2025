package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoConfig {
    // All in pedropathing coordinates
    // Field/robot constants

    public static boolean isRed = false;
    public static boolean isAudienceSide = true;
    // Vision (alliance goal tags)
    public static int APRIL_TAG_BLUE = 20;
    public static int APRIL_TAG_RED = 24;

    // Vision (scoring pattern tags)
    // GPP = 21, PGP = 22, PPG = 23
    public static int APRIL_TAG_GPP = 21;
    public static int APRIL_TAG_PGP = 22;
    public static int APRIL_TAG_PPG = 23;
}
