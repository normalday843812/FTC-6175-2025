package org.firstinspires.ftc.teamcode.managers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;

import androidx.annotation.NonNull;

import java.util.Locale;

/**
 * SpindexerModel - Central source of truth for spindexer state.
 *
 * <h2>Key Concepts</h2>
 * <ul>
 *   <li><b>Bucket</b>: Physical container on spindexer (0, 1, 2). Buckets ROTATE with the spindexer.</li>
 *   <li><b>Position</b>: Fixed location on chassis (0, 1, 2). Position 0 is at intake AND shooter.</li>
 *   <li><b>Sensors</b>: Fixed to chassis, watching positions. Sensor pairs 0-1, 2-3, 4-5 watch positions 0, 1, 2.</li>
 * </ul>
 *
 * <h2>Mental Model</h2>
 * <pre>
 * When spindexer rotates:
 *   - Buckets move to different positions
 *   - Sensors stay fixed (they see whatever bucket is now at their position)
 *   - Ball stays in its bucket (model doesn't change on rotation)
 *
 * Example:
 *   bucketContents = [GREEN, PURPLE, EMPTY]
 *   bucketAtFront = 0
 *   → Sensor at position 0 sees GREEN (bucket 0 is at front)
 *
 *   After rotate CW:
 *   bucketContents = [GREEN, PURPLE, EMPTY]  ← unchanged!
 *   bucketAtFront = 1
 *   → Sensor at position 0 sees PURPLE (bucket 1 is now at front)
 * </pre>
 *
 * <h2>Usage</h2>
 * <pre>
 * // Initialize
 * SpindexerModel model = new SpindexerModel();
 *
 * // When ball is intaked
 * model.onBallIntaked(detectedColor);
 *
 * // When spindexer rotates
 * model.rotateCW();  // or rotateCCW()
 * model.setBucketAtFront(bucketIndex);
 *
 * // When ball is shot
 * model.onBallShot();
 *
 * // Query state
 * int bucket = model.findBucketWithColor(BallColor.PURPLE);
 * int empty = model.findEmptyBucket();
 * BallColor next = model.getNextPatternColor();
 *
 * // Verify against sensors
 * model.verifyFrontBucket(sensors);
 * </pre>
 */
public class SpindexerModel {

    // -------------------------------------------------------------------------
    // Constants
    // -------------------------------------------------------------------------

    /**
     * Number of buckets on the spindexer
     */
    public static final int NUM_BUCKETS = 3;

    /**
     * Rotation direction constants
     */
    public static final int DIRECTION_CW = 1;
    public static final int DIRECTION_CCW = -1;

    // -------------------------------------------------------------------------
    // Ball State Enum (simplified - no color sorting)
    // -------------------------------------------------------------------------

    /**
     * Represents the contents of a bucket.
     */
    public enum BallColor {
        EMPTY,
        BALL
    }

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------

    /**
     * Contents of each bucket (index = bucket number, value = what's in it)
     */
    private final BallColor[] bucketContents = new BallColor[NUM_BUCKETS];

    /**
     * Which bucket is currently at position 0 (intake/shooter position)
     */
    private int bucketAtFront = 0;

    // Pattern tracking removed - no longer doing color sorting

    /**
     * Flag indicating if model may be out of sync with reality
     */
    private boolean needsVerification = true;

    /**
     * Last verification result
     */
    private VerificationResult lastVerification = null;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Creates a new SpindexerModel with all buckets empty.
     */
    public SpindexerModel() {
        reset();
    }

    /**
     * Resets all state to initial values.
     * Call this at the start of each OpMode.
     */
    public void reset() {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            bucketContents[i] = BallColor.EMPTY;
        }
        bucketAtFront = 0;
        needsVerification = true;
        lastVerification = null;
    }

    // -------------------------------------------------------------------------
    // Rotation Operations
    // -------------------------------------------------------------------------

    /**
     * Updates model after clockwise rotation (bucket indices increase at front).
     * Call this AFTER commanding the physical spindexer to rotate CW.
     */
    public void rotateCW() {
        bucketAtFront = (bucketAtFront + 1) % NUM_BUCKETS;
    }

    /**
     * Updates model after counter-clockwise rotation (bucket indices decrease at front).
     * Call this AFTER commanding the physical spindexer to rotate CCW.
     */
    public void rotateCCW() {
        bucketAtFront = (bucketAtFront - 1 + NUM_BUCKETS) % NUM_BUCKETS;
    }

    /**
     * Sets which bucket is at the front position.
     * Use this when commanding spindexer.setSlot() directly.
     *
     * @param bucket The bucket index (0-2) now at front
     */
    public void setBucketAtFront(int bucket) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            bucketAtFront = bucket;
        }
    }

    /**
     * Gets which bucket is currently at the front (intake/shooter) position.
     *
     * @return Bucket index (0-2) at front
     */
    public int getBucketAtFront() {
        return bucketAtFront;
    }

    /**
     * Gets which bucket is at a given physical position.
     *
     * @param position Physical position (0, 1, or 2)
     * @return Bucket index at that position
     */
    public int getBucketAtPosition(int position) {
        // If bucket B is at front (position 0), then:
        // - Position 0 has bucket B
        // - Position 1 has bucket (B+1)%3 for CW layout, or (B-1+3)%3 for CCW
        // Assuming CW layout where position increases = bucket index increases
        return (bucketAtFront + position) % NUM_BUCKETS;
    }

    // -------------------------------------------------------------------------
    // Ball Operations
    // -------------------------------------------------------------------------

    /**
     * Updates model when a ball is intaked.
     * Ball always enters at position 0 (front), into whichever bucket is there.
     */
    public void onBallIntaked() {
        bucketContents[bucketAtFront] = BallColor.BALL;
    }

    /**
     * Updates model when a ball is shot.
     * Ball always exits from position 0 (front), from whichever bucket is there.
     */
    public void onBallShot() {
        bucketContents[bucketAtFront] = BallColor.EMPTY;
    }

    /**
     * Directly sets the contents of a bucket.
     * Use for initialization or recovery from sensor verification.
     *
     * @param bucket Bucket index (0-2)
     * @param color  What's in the bucket
     */
    public void setBucketContents(int bucket, BallColor color) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            bucketContents[bucket] = color;
        }
    }

    /**
     * Gets the contents of a specific bucket.
     *
     * @param bucket Bucket index (0-2)
     * @return What's in the bucket
     */
    public BallColor getBucketContents(int bucket) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            return bucketContents[bucket];
        }
        return BallColor.EMPTY;
    }

    /**
     * Gets the contents of the bucket currently at front.
     *
     * @return What's in the front bucket
     */
    public BallColor getFrontBucketContents() {
        return bucketContents[bucketAtFront];
    }

    // -------------------------------------------------------------------------
    // Query Operations
    // -------------------------------------------------------------------------

    /**
     * Finds any bucket containing a ball.
     *
     * @return Bucket index (0-2) containing a ball, or -1 if all empty
     */
    public int findBallBucket() {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            if (bucketContents[i] == BallColor.BALL) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Finds an empty bucket.
     *
     * @return Bucket index (0-2) that is empty, or -1 if all full
     */
    public int findEmptyBucket() {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            if (bucketContents[i] == BallColor.EMPTY) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Finds the nearest empty bucket to the current front position.
     * "Nearest" means fewest rotations needed.
     *
     * @param preferredDirection DIRECTION_CW or DIRECTION_CCW for tie-breaking
     * @return Bucket index (0-2) that is empty, or -1 if all full
     */
    public int findNearestEmptyBucket(int preferredDirection) {
        // Check front first
        if (bucketContents[bucketAtFront] == BallColor.EMPTY) {
            return bucketAtFront;
        }

        // Check neighbors (1 rotation away)
        int cw = (bucketAtFront + 1) % NUM_BUCKETS;
        int ccw = (bucketAtFront - 1 + NUM_BUCKETS) % NUM_BUCKETS;

        boolean cwEmpty = bucketContents[cw] == BallColor.EMPTY;
        boolean ccwEmpty = bucketContents[ccw] == BallColor.EMPTY;

        if (cwEmpty && ccwEmpty) {
            return (preferredDirection == DIRECTION_CW) ? cw : ccw;
        } else if (cwEmpty) {
            return cw;
        } else if (ccwEmpty) {
            return ccw;
        }

        return -1; // All full
    }

    /**
     * Counts how many buckets contain balls (non-empty).
     *
     * @return Number of balls (0-3)
     */
    public int getBallCount() {
        int count = 0;
        for (BallColor c : bucketContents) {
            if (c == BallColor.BALL) {
                count++;
            }
        }
        return count;
    }

    /**
     * Checks if all buckets are full.
     *
     * @return true if no empty buckets
     */
    public boolean isFull() {
        return getBallCount() == NUM_BUCKETS;
    }

    /**
     * Checks if all buckets are empty.
     *
     * @return true if all buckets empty
     */
    public boolean isEmpty() {
        return getBallCount() == 0;
    }

    /**
     * Checks if a specific bucket is empty.
     *
     * @param bucket Bucket index (0-2)
     * @return true if that bucket is empty
     */
    public boolean isBucketEmpty(int bucket) {
        if (bucket >= 0 && bucket < NUM_BUCKETS) {
            return bucketContents[bucket] == BallColor.EMPTY;
        }
        return false;
    }

    /**
     * Calculates how many rotations needed to bring a bucket to front.
     *
     * @param targetBucket The bucket to bring to front
     * @param direction    DIRECTION_CW or DIRECTION_CCW
     * @return Number of rotations (0-2)
     */
    public int rotationsToFront(int targetBucket, int direction) {
        if (targetBucket == bucketAtFront) {
            return 0;
        }

        if (direction == DIRECTION_CW) {
            return (targetBucket - bucketAtFront + NUM_BUCKETS) % NUM_BUCKETS;
        } else {
            return (bucketAtFront - targetBucket + NUM_BUCKETS) % NUM_BUCKETS;
        }
    }

    /**
     * Determines the optimal rotation direction to bring a bucket to front.
     *
     * @param targetBucket The bucket to bring to front
     * @return DIRECTION_CW or DIRECTION_CCW (or 0 if already at front)
     */
    public int optimalDirection(int targetBucket) {
        if (targetBucket == bucketAtFront) {
            return 0;
        }

        int cwDist = rotationsToFront(targetBucket, DIRECTION_CW);
        int ccwDist = rotationsToFront(targetBucket, DIRECTION_CCW);

        return (cwDist <= ccwDist) ? DIRECTION_CW : DIRECTION_CCW;
    }

    // -------------------------------------------------------------------------
    // Sensor Verification
    // -------------------------------------------------------------------------

    /**
     * Result of sensor verification.
     */
    public static class VerificationResult {
        public final boolean matches;
        public final BallColor expected;
        public final BallColor actual;
        public final int bucket;

        public VerificationResult(boolean matches, BallColor expected, BallColor actual, int bucket) {
            this.matches = matches;
            this.expected = expected;
            this.actual = actual;
            this.bucket = bucket;
        }
    }

    /**
     * Verifies the front bucket against sensor reading.
     *
     * @param sensors The color sensor subsystem
     * @return VerificationResult indicating match or mismatch
     */
    public VerificationResult verifyFrontBucket(SlotColorSensors sensors) {
        BallColor expected = bucketContents[bucketAtFront];
        SlotColorSensors.BallColor sensorColor = sensors.getColor(0); // Position 0 = front
        BallColor actual = fromSensorColor(sensorColor);

        boolean matches = colorsMatch(expected, actual);
        lastVerification = new VerificationResult(matches, expected, actual, bucketAtFront);
        needsVerification = !matches;

        return lastVerification;
    }

    /**
     * Verifies all buckets against sensor readings.
     *
     * @param sensors The color sensor subsystem
     * @return true if all buckets match sensors
     */
    public boolean verifyAllBuckets(SlotColorSensors sensors) {
        boolean allMatch = true;

        for (int pos = 0; pos < NUM_BUCKETS; pos++) {
            int bucket = getBucketAtPosition(pos);
            BallColor expected = bucketContents[bucket];
            SlotColorSensors.BallColor sensorColor = sensors.getColor(pos);
            BallColor actual = fromSensorColor(sensorColor);

            if (!colorsMatch(expected, actual)) {
                allMatch = false;
                // Optionally auto-correct:
                // bucketContents[bucket] = actual;
            }
        }

        needsVerification = !allMatch;
        return allMatch;
    }

    /**
     * Rebuilds model state from sensor readings.
     * Use this to recover from a mismatch or initialize from unknown state.
     *
     * @param sensors The color sensor subsystem
     */
    public void rebuildFromSensors(SlotColorSensors sensors) {
        for (int pos = 0; pos < NUM_BUCKETS; pos++) {
            int bucket = getBucketAtPosition(pos);
            SlotColorSensors.BallColor sensorColor = sensors.getColor(pos);
            bucketContents[bucket] = fromSensorColor(sensorColor);
        }
        needsVerification = false;
    }

    /**
     * Checks if model needs verification (may be out of sync).
     *
     * @return true if verification recommended
     */
    public boolean needsVerification() {
        return needsVerification;
    }

    /**
     * Gets the last verification result.
     *
     * @return Last VerificationResult or null if never verified
     */
    public VerificationResult getLastVerification() {
        return lastVerification;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    /**
     * Adds model state to telemetry.
     *
     * @param telemetry The telemetry object
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== SPINDEXER MODEL ===");
        telemetry.addData("Bucket at front", bucketAtFront);
        telemetry.addData("Ball count", getBallCount());

        // Show bucket contents
        StringBuilder contents = new StringBuilder();
        for (int i = 0; i < NUM_BUCKETS; i++) {
            String marker = (i == bucketAtFront) ? "*" : " ";
            contents.append(String.format(Locale.US, "%s[%d]=%s ", marker, i, shortColor(bucketContents[i])));
        }
        telemetry.addData("Buckets", contents.toString().trim());

        // Show verification status
        if (needsVerification) {
            telemetry.addData("Status", "NEEDS VERIFICATION");
        }
        if (lastVerification != null && !lastVerification.matches) {
            telemetry.addData("Mismatch", String.format(Locale.US, "Bucket %d: expected %s, saw %s",
                    lastVerification.bucket,
                    shortColor(lastVerification.expected),
                    shortColor(lastVerification.actual)));
        }
    }

    // -------------------------------------------------------------------------
    // Utility Methods
    // -------------------------------------------------------------------------

    /**
     * Converts from SlotColorSensors.BallColor to internal BallColor.
     */
    private BallColor fromSensorColor(SlotColorSensors.BallColor sensorColor) {
        if (sensorColor == SlotColorSensors.BallColor.BALL) {
            return BallColor.BALL;
        }
        return BallColor.EMPTY;
    }

    /**
     * Converts from internal BallColor to SlotColorSensors.BallColor.
     */
    public SlotColorSensors.BallColor toSensorColor(BallColor color) {
        if (color == BallColor.BALL) {
            return SlotColorSensors.BallColor.BALL;
        }
        return SlotColorSensors.BallColor.NONE;
    }

    /**
     * Checks if two states match.
     */
    private boolean colorsMatch(BallColor expected, BallColor actual) {
        return expected == actual;
    }

    /**
     * Gets a short string representation of a state for telemetry.
     */
    private String shortColor(BallColor color) {
        if (color == null) return "?";
        switch (color) {
            case EMPTY:
                return "---";
            case BALL:
                return "BAL";
            default:
                return "???";
        }
    }

    // -------------------------------------------------------------------------
    // Debug / Testing
    // -------------------------------------------------------------------------

    /**
     * Returns a string representation of the current state.
     */
    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("SpindexerModel{");
        sb.append("front=").append(bucketAtFront);
        sb.append(", buckets=[");
        for (int i = 0; i < NUM_BUCKETS; i++) {
            if (i > 0) sb.append(", ");
            sb.append(shortColor(bucketContents[i]));
        }
        sb.append("]");
        sb.append("}");
        return sb.toString();
    }
}