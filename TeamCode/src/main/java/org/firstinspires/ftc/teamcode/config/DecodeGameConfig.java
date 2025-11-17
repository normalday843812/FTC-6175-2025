package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_GPP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PGP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PPG;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class DecodeGameConfig {
    // Motif patterns: true = PURPLE, false = GREEN
    public static boolean[] PATTERN_GPP = new boolean[]{false, true, true};
    public static boolean[] PATTERN_PGP = new boolean[]{true, false, true};
    public static boolean[] PATTERN_PPG = new boolean[]{true, true, false};

    public static boolean[] patternForTag(int tagId) {
        if (tagId == APRIL_TAG_GPP) return PATTERN_GPP;
        if (tagId == APRIL_TAG_PGP) return PATTERN_PGP;
        if (tagId == APRIL_TAG_PPG) return PATTERN_PPG;
        return null;
    }

    public static Pose[] INTAKE_SETS_RED = new Pose[]{
            new Pose(103, 84, Math.toRadians(0)),
            new Pose(103, 60, Math.toRadians(0)),
            new Pose(103, 35, Math.toRadians(0))
    };
    public static Pose[] INTAKE_SETS_BLUE = new Pose[]{
            new Pose(40, 84, Math.toRadians(180)),
            new Pose(40, 60, Math.toRadians(180)),
            new Pose(40, 35, Math.toRadians(180))
    };

    public static Pose startPose(boolean isRed, boolean audienceSide) {
        return isRed
                ? (audienceSide ? new Pose(88, 7.5, Math.toRadians(90)) : new Pose(117.5, 131.2, Math.toRadians(-142)))
                : (audienceSide ? new Pose(56, 7.5, Math.toRadians(90)) : new Pose(27.29, 131.2, Math.toRadians(-36)));
    }

    public static Pose shootPose(boolean isRed) {
        return isRed ? new Pose(115, 120, Math.toRadians(45))
                : new Pose(29, 120, Math.toRadians(135));
    }

    public static Pose finalPose(boolean isRed) {
        return isRed ? new Pose(106, 12, Math.toRadians(180))
                : new Pose(36, 12, Math.toRadians(0));
    }
}