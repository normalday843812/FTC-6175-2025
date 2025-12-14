package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.AUTO_PATTERN_MIN_CONFIDENCE;
import static org.firstinspires.ftc.teamcode.config.LLAprilTagConfig.AUTO_PATTERN_SCORE_DECAY_S;

import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

/**
 * Tracks the most likely autonomous scoring pattern AprilTag (GPP/PGP/PPG) over time.
 *
 * <p>Maintains a decaying score per candidate tag so we can keep a "best guess" even when the
 * tag is not currently fresh.</p>
 */
public final class AutoPatternDetector {

    private static final class YawStats {
        private long n = 0;
        private double mean = 0.0;
        private double m2 = 0.0;

        void push(double x) {
            n++;
            double delta = x - mean;
            mean += delta / n;
            double delta2 = x - mean;
            m2 += delta * delta2;
        }

        double stddev() {
            if (n < 2) return 0.0;
            return Math.sqrt(m2 / (n - 1));
        }
    }

    private final LLAprilTag ll;
    private final int[] ids = new int[]{
            AutoConfig.APRIL_TAG_GPP,
            AutoConfig.APRIL_TAG_PGP,
            AutoConfig.APRIL_TAG_PPG
    };
    private final double[] scores = new double[ids.length];
    private final YawStats[] yawStats = new YawStats[ids.length];

    private long lastUpdateMs = System.currentTimeMillis();

    private int bestTagId = -1;
    private double bestScore = 0.0;
    private double confidence = 0.0;

    public AutoPatternDetector(LLAprilTag llAprilTag) {
        this.ll = llAprilTag;
        for (int i = 0; i < yawStats.length; i++) {
            yawStats[i] = new YawStats();
        }
    }

    public void update() {
        if (ll == null) return;

        long now = System.currentTimeMillis();
        long dtMs = Math.max(0L, now - lastUpdateMs);
        lastUpdateMs = now;

        // Exponential decay over time so old detections fade out.
        double tauMs = Math.max(1.0, AUTO_PATTERN_SCORE_DECAY_S * 1000.0);
        double decay = Math.exp(-dtMs / tauMs);
        for (int i = 0; i < scores.length; i++) {
            scores[i] *= decay;
        }

        // Add evidence for any currently-fresh tag.
        for (int i = 0; i < ids.length; i++) {
            LLAprilTag.YawInfo info = ll.getYawInfoForTag(ids[i]);
            if (!info.fresh) continue;

            if (!Double.isNaN(info.avgDeg)) {
                yawStats[i].push(info.avgDeg);
            }
            double yawStd = yawStats[i].stddev();

            // Heuristic: area is our primary evidence; divide by (1 + yaw stddev) so stable sightings are preferred.
            double evidence = Math.max(0.0, info.area) / (1.0 + yawStd);
            scores[i] += evidence;
        }

        // Pick best + compute confidence vs. second best.
        int bestIdx = -1;
        int secondIdx = -1;
        double best = -1.0;
        double second = -1.0;
        for (int i = 0; i < scores.length; i++) {
            double s = scores[i];
            if (s > best) {
                second = best;
                secondIdx = bestIdx;
                best = s;
                bestIdx = i;
            } else if (s > second) {
                second = s;
                secondIdx = i;
            }
        }

        bestTagId = (bestIdx >= 0) ? ids[bestIdx] : -1;
        bestScore = Math.max(0.0, best);
        double secondScore = Math.max(0.0, second);
        confidence = (bestScore <= 0.0)
                ? 0.0
                : (bestScore / (bestScore + secondScore + 1e-9));
    }

    public int getBestTagId() {
        return bestTagId;
    }

    public double getBestScore() {
        return bestScore;
    }

    public double getConfidence() {
        return confidence;
    }

    public boolean isConfident() {
        return confidence >= AUTO_PATTERN_MIN_CONFIDENCE && bestTagId >= 0;
    }

    public void addTelemetry(TelemetryHelper tele, boolean patternApplied) {
        if (tele == null) return;

        tele.addLine("=== AUTO PATTERN TAG ===")
                .addData("Applied", "%b", patternApplied)
                .addData("Best", "%s", bestTagId < 0 ? "---" : (DecodeGameConfig.patternNameForTag(bestTagId) + " (" + bestTagId + ")"))
                .addData("Conf", "%.2f", confidence)
                .addData("Score", "%.3f", bestScore);

        for (int i = 0; i < ids.length; i++) {
            int id = ids[i];
            LLAprilTag.YawInfo info = ll.getYawInfoForTag(id);
            tele.addData("Tag " + DecodeGameConfig.patternNameForTag(id),
                    "fresh=%b age=%dms area=%.3f yaw=%.1f score=%.3f",
                    info.fresh,
                    info.ageMs,
                    info.area,
                    Double.isNaN(info.avgDeg) ? 0.0 : info.avgDeg,
                    scores[i]);
        }
    }
}

