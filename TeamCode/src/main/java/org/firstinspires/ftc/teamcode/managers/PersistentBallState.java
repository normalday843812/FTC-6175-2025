package org.firstinspires.ftc.teamcode.managers;

/**
 * Static holder for ball state that persists across OpModes.
 *
 * This allows ball positions to carry over from auto to teleop.
 * Use ResetBallState OpMode to reset when needed.
 */
public class PersistentBallState {

    public static final int NUM_BUCKETS = 3;

    // Static state that persists across OpMode runs
    private static final boolean[] hasBall = new boolean[NUM_BUCKETS];
    private static int bucketAtFront = 0;
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
            return hasBall[bucket];
        }
        return false;
    }

    /**
     * Set whether a bucket has a ball.
     */
    public static void setBall(int bucket, boolean ball) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            hasBall[bucket] = ball;
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
        for (boolean b : hasBall) {
            if (b) count++;
        }
        return count;
    }

    /**
     * Reset all state to empty.
     */
    public static void reset() {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            hasBall[i] = false;
        }
        bucketAtFront = 0;
        initialized = false;
    }

    /**
     * Copy state from a SpindexerModel.
     */
    public static void saveFromModel(SpindexerModel model) {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            hasBall[i] = model.getBucketContents(i) == SpindexerModel.BallColor.BALL;
        }
        bucketAtFront = model.getBucketAtFront();
        initialized = true;
    }

    /**
     * Load state into a SpindexerModel.
     */
    public static void loadIntoModel(SpindexerModel model) {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            model.setBucketContents(i, hasBall[i] ? SpindexerModel.BallColor.BALL : SpindexerModel.BallColor.EMPTY);
        }
        model.setBucketAtFront(bucketAtFront);
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
            sb.append(marker).append(i).append(":").append(hasBall[i] ? "BALL" : "---");
        }
        sb.append("]");
        return sb.toString();
    }
}
