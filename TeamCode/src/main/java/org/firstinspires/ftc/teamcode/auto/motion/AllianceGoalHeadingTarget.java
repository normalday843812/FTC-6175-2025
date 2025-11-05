package org.firstinspires.ftc.teamcode.auto.motion;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FALLBACK_HEADING_BLUE_DEG;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FALLBACK_HEADING_RED_DEG;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.vision.LLAprilTag;

public class AllianceGoalHeadingTarget implements HeadingTarget {
    private final LLAprilTag ll;
    private final boolean isRed;

    public AllianceGoalHeadingTarget(LLAprilTag ll, boolean isRed) {
        this.ll = ll;
        this.isRed = isRed;
    }

    @Override
    public double getTargetHeadingDeg(Pose currentPose) {
        LLAprilTag.YawInfo info = ll.getYawInfoForAllianceHome(isRed);
        double currentDeg = Math.toDegrees(currentPose.getHeading());

        if (info.fresh && !Double.isNaN(info.avgDeg)) {
            return normalizeDeg(currentDeg + info.avgDeg);
        } else {
            return isRed ? FALLBACK_HEADING_RED_DEG : FALLBACK_HEADING_BLUE_DEG;
        }
    }


    @Override
    public String debugName() {
        return "AllianceGoalHeadingTarget";
    }

    private static double normalizeDeg(double a) {
        while (a > 360) a -= 360;
        while (a < 0) a += 360;
        return a;
    }
}
