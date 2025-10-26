package org.firstinspires.ftc.teamcode.auto.motion;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.FALLBACK_HEADING_BLUE_DEG;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.FALLBACK_HEADING_RED_DEG;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class AllianceGoalHeadingTarget implements HeadingTarget {
    private final Limelight3A limelight;
    private final boolean isRed;

    private volatile double lastSeenYawDeg = Double.NaN;
    private volatile long lastSeenTimeMs = 0L;
    private static final long LL_TTL_MS = 200;

    public AllianceGoalHeadingTarget(Limelight3A limelight, boolean isRed) {
        this.limelight = limelight;
        this.isRed = isRed;
    }

    @Override
    public double getTargetHeadingDeg(Pose currentPose) {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            int goalId = isRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
            double yawDeg = findYawDegToTag(r, goalId);
            if (!Double.isNaN(yawDeg)) {
                lastSeenYawDeg = yawDeg;
                lastSeenTimeMs = System.currentTimeMillis();
            }
        }
        boolean fresh = (System.currentTimeMillis() - lastSeenTimeMs) <= LL_TTL_MS;
        if (fresh && !Double.isNaN(lastSeenYawDeg)) {
            double currentDeg = Math.toDegrees(currentPose.getHeading());
            return normalizeDeg(currentDeg + lastSeenYawDeg);
        } else {
            return isRed ? FALLBACK_HEADING_RED_DEG : FALLBACK_HEADING_BLUE_DEG;
        }
    }

    @Override
    public String debugName() {
        return "AllianceGoalHeadingTarget";
    }

    private static double findYawDegToTag(LLResult result, int tagId) {
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) return Double.NaN;

        for (LLResultTypes.FiducialResult f : tags) {
            if (f.getFiducialId() == tagId) {
                if (f.getTargetPoseCameraSpace() == null || f.getTargetPoseCameraSpace().getPosition() == null) {
                    return Double.NaN;
                }
                double x = f.getTargetPoseCameraSpace().getPosition().x;
                double z = f.getTargetPoseCameraSpace().getPosition().z;
                double yawRad = Math.atan2(x, z); // Left is positive
                return Math.toDegrees(yawRad);
            }
        }
        return Double.NaN;
    }

    private static double normalizeDeg(double a) {
        while (a > 360) a -= 360;
        while (a < 0) a += 360;
        return a;
    }
}
