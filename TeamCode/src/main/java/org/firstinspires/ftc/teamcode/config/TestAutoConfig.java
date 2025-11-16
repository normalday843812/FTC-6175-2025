package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class TestAutoConfig {
    private TestAutoConfig() {
    }

    public static boolean ENABLE_PATTERN_SEEK = true;
    public static boolean ENABLE_INTAKE = true;
    public static boolean ENABLE_DEPOSIT = true;
    public static boolean ENABLE_FINAL_MOVE = true;
    public static boolean USE_COLOR_SENSORS = true;
    public static boolean USE_DISTANCE_SENSOR = false;
    public static boolean RUN_DEPOSIT_ROUTE = true;
    public static boolean USE_UI_LIGHT = true;

    public static boolean FORCE_RED = AutoConfig.isRed;
    public static boolean FORCE_AUDIENCE_SIDE = AutoConfig.isAudienceSide;
}
