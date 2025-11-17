package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class AutoDepositConfig {
    public static Pose pickShootPose(boolean isRed, boolean isAudienceSide) {
        if (isRed && isAudienceSide) {
            return new Pose(100, 100, Math.toRadians(45));
        } else if (!isRed && isAudienceSide) {
            return new Pose(42, 105, Math.toRadians(135));
        } else if (isRed) {
            return new Pose(100, 100, Math.toRadians(45));
        } else {
            return new Pose(42, 105, Math.toRadians(135));
        }
    }
}
