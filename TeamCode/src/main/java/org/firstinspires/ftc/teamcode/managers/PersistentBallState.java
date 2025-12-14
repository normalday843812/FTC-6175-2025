package org.firstinspires.ftc.teamcode.managers;

import java.util.Arrays;

/**
 * Static holder for ball state that persists across OpModes.
 * This allows ball positions to carry over from auto to teleop.
 * Use ResetBallState OpMode to reset when needed.
 */
public class PersistentBallState {

    public static final int NUM_BUCKETS = 3;

    // Static state that persists across OpMode runs
    private static final SpindexerModel.BallColor[] bucketContents = new SpindexerModel.BallColor[NUM_BUCKETS];
    private static int bucketAtFront = 0;
    private static SpindexerModel.BallColor[] pattern = null;
    private static int patternIndex = 0;
    private static boolean initialized = false;

    /**
     * Check if state has been initialized this session.
     */
    public static boolean isInitialized() {
        return initialized;
    }

    /**
     * Mark state as initialized.
     */
    public static void markInitialized() {
        initialized = true;
    }

    /**
     * Get whether a bucket has a ball.
     */
    public static boolean hasBall(int bucket) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            SpindexerModel.BallColor c = bucketContents[bucket];
            return c != null && c != SpindexerModel.BallColor.EMPTY;
        }
        return false;
    }

    /**
     * Get the stored contents of a bucket.
     */
    public static SpindexerModel.BallColor getBucketContents(int bucket) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            SpindexerModel.BallColor c = bucketContents[bucket];
            return c == null ? SpindexerModel.BallColor.EMPTY : c;
        }
        return SpindexerModel.BallColor.EMPTY;
    }

    /**
     * Set the stored contents of a bucket.
     */
    public static void setBucketContents(int bucket, SpindexerModel.BallColor color) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            bucketContents[bucket] = (color == null) ? SpindexerModel.BallColor.UNKNOWN : color;
        }
    }

    /**
     * Get which bucket is at front.
     */
    public static int getBucketAtFront() {
        return bucketAtFront;
    }

    /**
     * Set which bucket is at front.
     */
    public static void setBucketAtFront(int bucket) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            bucketAtFront = bucket;
        }
    }

    /**
     * Get the total ball count.
     */
    public static int getBallCount() {
        int count = 0;
        for (SpindexerModel.BallColor c : bucketContents) {
            if (c != null && c != SpindexerModel.BallColor.EMPTY) count++;
        }
        return count;
    }

    /**
     * Reset all state to empty.
     */
    public static void reset() {
        Arrays.fill(bucketContents, SpindexerModel.BallColor.EMPTY);
        bucketAtFront = 0;
        pattern = null;
        patternIndex = 0;
        initialized = false;
    }

    /**
     * Copy state from a SpindexerModel.
     */
    public static void saveFromModel(SpindexerModel model) {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            bucketContents[i] = model.getBucketContents(i);
        }
        bucketAtFront = model.getBucketAtFront();
        pattern = model.getPattern();
        patternIndex = model.getPatternIndex();
        initialized = true;
    }

    /**
     * Load state into a SpindexerModel.
     */
    public static void loadIntoModel(SpindexerModel model) {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            model.setBucketContents(i, getBucketContents(i));
        }
        model.setBucketAtFront(bucketAtFront);
        if (pattern != null) {
            model.setPattern(pattern);
            model.setPatternProgress(patternIndex);
        } else {
            model.clearPattern();
        }
    }

    /**
     * Get a string representation of current state for telemetry.
     */
    public static String getStateString() {
        StringBuilder sb = new StringBuilder();
        sb.append("[");
        for (int i = 0; i < NUM_BUCKETS; i++) {
            if (i > 0) sb.append(", ");
            String marker = (i == bucketAtFront) ? "*" : "";
            sb.append(marker).append(i).append(":").append(getBucketContents(i).name());
        }
        sb.append("]");
        return sb.toString();
    }
}
