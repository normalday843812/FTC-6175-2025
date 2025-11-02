package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_BLUE;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_GPP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PGP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PPG;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_RED;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.MAX_TAG_DIST_M;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.MIN_TAG_AREA;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.TTL_MS;

import com.pedropathing.control.LowPassFilter;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class LLAprilTag {
    public static Integer SELECTED_PATTERN_TAG_ID = null;

    public static final class YawInfo {
        public final double rawDeg;
        public final double avgDeg;
        public final boolean fresh;
        public final double distanceM;
        public final double area;
        public final long ageMs;

        public YawInfo(double rawDeg, double avgDeg, boolean fresh, double distanceM, double area, long ageMs) {
            this.rawDeg = rawDeg;
            this.avgDeg = avgDeg;
            this.fresh = fresh;
            this.distanceM = distanceM;
            this.area = area;
            this.ageMs = ageMs;
        }
    }

    private static final class TagState {
        double rawDeg = Double.NaN;
        double avgDeg = Double.NaN;
        double lastArea = 0.0;
        double lastDist = Double.POSITIVE_INFINITY;
        long lastSeenMs = 0L;
        private final LowPassFilter yawFilter = new LowPassFilter(YAW_FILTER_ALPHA);

        void push(double yaw, double area, double dist, long now) {
            rawDeg = yaw;
            lastArea = area;
            lastDist = dist;
            yawFilter.update(yaw, 0.0);
            avgDeg = yawFilter.getState();
            lastSeenMs = now;
        }

        YawInfo asInfo(long now) {
            boolean fresh = (now - lastSeenMs) <= TTL_MS;
            long age = lastSeenMs == 0L ? Long.MAX_VALUE : (now - lastSeenMs);
            return new YawInfo(rawDeg, avgDeg, fresh, lastDist, lastArea, age);
        }
    }

    private static final double YAW_FILTER_ALPHA = 0.2; // tune

    private final Limelight3A limelight;
    private final TelemetryHelper tele;

    private double selectedPatternConfidence = 0.0;

    private final Map<Integer, TagState> states = new HashMap<>();
    private LLResult result;

    public LLAprilTag(Limelight3A limelight, OpMode opMode) {
        this.limelight = limelight;
        this.tele = new TelemetryHelper(opMode, TELEMETRY_ENABLED);
    }

    public void update() {
        result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            addTelemetry();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags != null) {
            long now = System.currentTimeMillis();

            for (LLResultTypes.FiducialResult f : tags) {
                int id = f.getFiducialId();

                double yaw = yawDeg(f);
                double area = safeArea(f);
                double dist = distanceMeters(f);

                boolean areaOk = area >= MIN_TAG_AREA;
                boolean distOk = dist <= MAX_TAG_DIST_M;
                if (!Double.isNaN(yaw) && (areaOk || distOk)) {
                    states.computeIfAbsent(id, k -> new TagState())
                            .push(yaw, area, dist, now);
                }

                if (id == APRIL_TAG_GPP || id == APRIL_TAG_PGP || id == APRIL_TAG_PPG) {
                    double conf = confidenceFromDistance(f);
                    if (conf > selectedPatternConfidence) {
                        selectedPatternConfidence = conf;
                        SELECTED_PATTERN_TAG_ID = id;
                    }
                }
            }
        }

        addTelemetry();
    }

    public Pose3D getBotPose() {
        return (result != null && result.isValid()) ? result.getBotpose() : null;
    }

    private static double confidenceFromDistance(LLResultTypes.FiducialResult f) {
        double meters = distanceMeters(f);
        if (!Double.isFinite(meters)) return 0.0;
        return 1.0 / (1.0 + meters);
    }

    public YawInfo getYawInfoForTag(int tagId) {
        long now = System.currentTimeMillis();
        TagState st = states.get(tagId);
        return (st == null)
                ? new YawInfo(Double.NaN, Double.NaN, false, Double.POSITIVE_INFINITY, 0.0, Long.MAX_VALUE)
                : st.asInfo(now);
    }

    public YawInfo getYawInfoForAllianceHome(boolean allianceRed) {
        int id = allianceRed ? APRIL_TAG_RED : APRIL_TAG_BLUE;
        return getYawInfoForTag(id);
    }

    private static double yawDeg(LLResultTypes.FiducialResult f) {
        if (f == null || f.getTargetPoseCameraSpace() == null || f.getTargetPoseCameraSpace().getPosition() == null) {
            return Double.NaN;
        }
        double x = f.getTargetPoseCameraSpace().getPosition().x;
        double z = f.getTargetPoseCameraSpace().getPosition().z;
        return Math.toDegrees(Math.atan2(x, z));
    }

    private static double distanceMeters(LLResultTypes.FiducialResult f) {
        if (f == null || f.getTargetPoseCameraSpace() == null || f.getTargetPoseCameraSpace().getPosition() == null) {
            return Double.POSITIVE_INFINITY;
        }
        double x = f.getTargetPoseCameraSpace().getPosition().x;
        double y = f.getTargetPoseCameraSpace().getPosition().y;
        double z = f.getTargetPoseCameraSpace().getPosition().z;
        return Math.sqrt(x * x + y * y + z * z);
    }

    private static double safeArea(LLResultTypes.FiducialResult f) {
        try {
            return f.getTargetArea();
        } catch (Throwable t) {
            return 0.0;
        }
    }

    private void addTelemetry() {
        boolean poseAvailable = result != null && result.getBotpose() != null;

        tele.addLine("=== LL APRILTAG ===")
                .addData("Pattern ID", () -> SELECTED_PATTERN_TAG_ID)
                .addData("Pattern Confidence", "%.3f", selectedPatternConfidence)
                .addData("Pose Set", poseAvailable ? "yes" : "no");

        if (poseAvailable) {
            Pose3D botpose = result.getBotpose();
            tele.addData("LL x", "%.2f", botpose.getPosition().x)
                    .addData("LL y", "%.2f", botpose.getPosition().y)
                    .addData("LL yaw", "%.2f", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        YawInfo r = getYawInfoForTag(APRIL_TAG_RED);
        YawInfo b = getYawInfoForTag(APRIL_TAG_BLUE);
        tele.addData("RedYaw(avg)", "%.1f", Double.isNaN(r.avgDeg) ? 0.0 : r.avgDeg)
                .addData("RedFresh", "%b", r.fresh)
                .addData("BlueYaw(avg)", "%.1f", Double.isNaN(b.avgDeg) ? 0.0 : b.avgDeg)
                .addData("BlueFresh", "%b", b.fresh);
    }
}
