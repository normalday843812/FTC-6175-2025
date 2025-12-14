package org.firstinspires.ftc.teamcode.managers;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.Locale;

/**
 * SpindexerModel - Central source of truth for spindexer state.
 *
 * <h2>Key Concepts</h2>
 * <ul>
 *   <li><b>Bucket</b>: A physical container on the spindexer (0, 1, 2). Buckets rotate with the spindexer.</li>
 *   <li><b>Position</b>: A fixed chassis location (0, 1, 2). Position 0 is the front (intake + shooter).</li>
 *   <li><b>Sensors</b>: Fixed to the chassis. This robot intentionally mounts all color sensors at the front (position 0).</li>
 * </ul>
 *
 * <h2>Mental Model</h2>
 * <pre>
 * When spindexer rotates:
 *   - Buckets move to different positions
 *   - Sensors stay fixed (they see whatever bucket is now at their position)
 *   - Ball stays in its bucket (model doesn't change on rotation)
 *
 * Example (matches {@link org.firstinspires.ftc.teamcode.subsystems.Spindexer#stepForward()} wrap behavior):
 * <pre>
 * slot0(front)=EMPTY, slot1=GREEN, slot2=PURPLE
 * stepForward()  (2 → 0 wrap)
 * slot0(front)=PURPLE, slot1=EMPTY, slot2=GREEN
 * </pre>
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
    // Ball Color Enum
    // -------------------------------------------------------------------------

    /**
     * Represents the contents of a bucket.
     *
     * <p>Notes:
     * <ul>
     *   <li>{@link #EMPTY} means no ball in the bucket.</li>
     *   <li>{@link #UNKNOWN} means a ball is present but its color has not been identified yet.</li>
     * </ul>
     */
    public enum BallColor {
        EMPTY,
        PURPLE,
        GREEN,
        UNKNOWN
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

    /**
     * Desired shooting pattern (order of colors to shoot), or null if unknown/not used.
     */
    private BallColor[] pattern = null;

    /**
     * How many shots have been taken in the current pattern.
     */
    private int patternIndex = 0;

    /**
     * AprilTag ID used to derive the current pattern, or -1 if unknown/manual.
     */
    private int patternTagId = -1;

    /**
     * Confidence (0..1) for {@link #patternTagId}.
     */
    private double patternConfidence = 0.0;

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
        Arrays.fill(bucketContents, BallColor.EMPTY);
        bucketAtFront = 0;
        pattern = null;
        patternIndex = 0;
        patternTagId = -1;
        patternConfidence = 0.0;
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
     * Gets which bucket is at a given physical position (slot) on the chassis.
     *
     * <p>Important: "stepForward" (commanded slot +1, wrapping 2→0) must match your physical
     * permutation where the old slot-2 bucket becomes the new front (slot-0). To make that
     * true while keeping {@code bucketAtFront == spindexer.getCommandedSlot()}, the mapping
     * here uses {@code bucketAtFront - position}.</p>
     *
     * @param position Physical position (0 = front, 1, or 2)
     * @return Bucket index (0-2) at that position
     */
    public int getBucketAtPosition(int position) {
        // If bucket B is at the front (position 0), then:
        // - Position 0 has bucket B
        // - Position 1 has bucket (B-1)
        // - Position 2 has bucket (B-2)
        //
        // With this mapping, "stepForward" (bucketAtFront++) yields:
        //   new slot0 = old slot2
        //   new slot1 = old slot0
        //   new slot2 = old slot1
        return (bucketAtFront - position + NUM_BUCKETS) % NUM_BUCKETS;
    }

    // -------------------------------------------------------------------------
    // Ball Operations
    // -------------------------------------------------------------------------

    /**
     * Updates model when a ball is intaked.
     * Ball always enters at position 0 (front), into whichever bucket is there.
     */
    public void onBallIntaked() {
        bucketContents[bucketAtFront] = BallColor.UNKNOWN;
    }

    /**
     * Updates model when a ball is shot.
     * Ball always exits from position 0 (front), from whichever bucket is there.
     */
    public void onBallShot() {
        bucketContents[bucketAtFront] = BallColor.EMPTY;
        if (pattern != null && patternIndex < pattern.length) {
            patternIndex++;
        }
    }

    /**
     * Updates model when a ball is intaked with a known color.
     */
    public void onBallIntaked(BallColor color) {
        bucketContents[bucketAtFront] = (color == null) ? BallColor.UNKNOWN : color;
    }

    /**
     * Updates model when a ball is intaked using sensor output.
     */
    public void onBallIntaked(SlotColorSensors.BallColor sensorColor) {
        onBallIntaked(fromSensorColor(sensorColor));
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
     * Finds a bucket containing a specific color.
     *
     * @param color The color to find
     * @return Bucket index (0-2) containing that color, or -1 if not found
     */
    public int findBucketWithColor(BallColor color) {
        if (color == null) return -1;
        for (int i = 0; i < NUM_BUCKETS; i++) {
            if (bucketContents[i] == color) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Finds a bucket containing a specific color (using sensor color type).
     */
    public int findBucketWithColor(SlotColorSensors.BallColor sensorColor) {
        return findBucketWithColor(fromSensorColor(sensorColor));
    }

    /**
     * Finds any bucket containing a ball.
     *
     * @return Bucket index (0-2) containing a ball, or -1 if all empty
     */
    public int findBallBucket() {
        for (int i = 0; i < NUM_BUCKETS; i++) {
            if (bucketContents[i] != BallColor.EMPTY) {
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
            if (c != BallColor.EMPTY) {
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
     * Returns true if any bucket contains a ball whose color is not yet identified.
     */
    public boolean hasUnknownBalls() {
        for (BallColor c : bucketContents) {
            if (c == BallColor.UNKNOWN) {
                return true;
            }
        }
        return false;
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
    // Pattern Operations
    // -------------------------------------------------------------------------

    /**
     * Sets the desired shooting pattern.
     *
     * @param colors Array of colors in the order they should be shot (e.g. [PURPLE, GREEN, PURPLE])
     */
    public void setPattern(BallColor... colors) {
        if (colors == null || colors.length == 0) {
            pattern = null;
            patternIndex = 0;
            patternTagId = -1;
            patternConfidence = 0.0;
            return;
        }
        pattern = colors.clone();
        patternIndex = 0;
    }

    /**
     * Sets the desired shooting pattern from sensor color types.
     */
    public void setPattern(SlotColorSensors.BallColor... colors) {
        if (colors == null || colors.length == 0) {
            pattern = null;
            patternIndex = 0;
            patternTagId = -1;
            patternConfidence = 0.0;
            return;
        }
        pattern = new BallColor[colors.length];
        for (int i = 0; i < colors.length; i++) {
            pattern[i] = fromSensorColor(colors[i]);
        }
        patternIndex = 0;
    }

    public void setPatternMeta(int tagId, double confidence01) {
        patternTagId = tagId;
        patternConfidence = Math.max(0.0, Math.min(1.0, confidence01));
    }

    public boolean isPatternKnown() {
        return pattern != null && pattern.length > 0;
    }

    /**
     * Returns a copy of the current pattern, or null if none is set.
     */
    public BallColor[] getPattern() {
        return pattern == null ? null : pattern.clone();
    }

    public int getPatternIndex() {
        return patternIndex;
    }

    public int getPatternTagId() {
        return patternTagId;
    }

    public double getPatternConfidence() {
        return patternConfidence;
    }

    /**
     * Sets the current pattern progress (how many shots have already been taken in this pattern).
     */
    public void setPatternProgress(int index) {
        if (pattern == null) {
            patternIndex = 0;
            return;
        }
        patternIndex = Math.max(0, Math.min(index, pattern.length));
    }

    /**
     * Gets the next color to shoot according to the pattern.
     *
     * @return Next color needed, or null if pattern is unknown or complete
     */
    public BallColor getNextPatternColor() {
        if (pattern == null || patternIndex >= pattern.length) {
            return null;
        }
        return pattern[patternIndex];
    }

    public int getRemainingPatternShots() {
        if (pattern == null) {
            return 0;
        }
        return Math.max(0, pattern.length - patternIndex);
    }

    public boolean isPatternComplete() {
        return pattern == null || patternIndex >= pattern.length;
    }

    /**
     * Finds the bucket containing the next pattern color.
     *
     * @return Bucket index (0-2) or -1 if not found or no pattern
     */
    public int findNextPatternBucket() {
        BallColor needed = getNextPatternColor();
        if (needed == null) {
            return -1;
        }
        return findBucketWithColor(needed);
    }

    public void resetPatternProgress() {
        patternIndex = 0;
    }

    public void clearPattern() {
        pattern = null;
        patternIndex = 0;
        patternTagId = -1;
        patternConfidence = 0.0;
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
        BallColor actual = (sensors != null) ? fromSensorColor(sensors.getFrontColor()) : BallColor.UNKNOWN;

        boolean matches = colorsMatch(expected, actual);
        lastVerification = new VerificationResult(matches, expected, actual, bucketAtFront);
        needsVerification = !matches;

        return lastVerification;
    }

    /**
     * Verifies all buckets against sensor readings.
     * Note: this robot intentionally mounts all color sensors at the front (slot 0), so we can only
     * directly verify the bucket currently at the front. Full-bucket verification requires rotating
     * the spindexer and sampling at slot 0.
     * @param sensors The color sensor subsystem
     * @return true if all buckets match sensors
     */
    public boolean verifyAllBuckets(SlotColorSensors sensors) {
        return verifyFrontBucket(sensors).matches;
    }

    /**
     * Rebuilds model state from sensor readings.
     * Note: this robot intentionally mounts all color sensors at the front (slot 0), so the only
     * bucket we can rebuild without rotating is the bucket currently at the front.
     * Use this to recover the front bucket from sensor truth (other buckets remain unchanged).
     * @param sensors The color sensor subsystem
     */
    public void rebuildFromSensors(SlotColorSensors sensors) {
        bucketContents[bucketAtFront] = (sensors != null) ? fromSensorColor(sensors.getFrontColor()) : BallColor.UNKNOWN;
        needsVerification = false;
        lastVerification = null;
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

        if (pattern != null) {
            StringBuilder pat = new StringBuilder();
            for (int i = 0; i < pattern.length; i++) {
                String marker = (i == patternIndex) ? ">" : " ";
                pat.append(String.format(Locale.US, "%s%s ", marker, shortColor(pattern[i])));
            }
            telemetry.addData("Pattern", pat.toString().trim());
            telemetry.addData("Next shot", getNextPatternColor() == null ? "---" : shortColor(getNextPatternColor()));
        }

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
        if (sensorColor == null) return BallColor.UNKNOWN;
        switch (sensorColor) {
            case PURPLE:
                return BallColor.PURPLE;
            case GREEN:
                return BallColor.GREEN;
            case NONE:
                return BallColor.EMPTY;
            default:
                return BallColor.UNKNOWN;
        }
    }

    /**
     * Converts from internal BallColor to SlotColorSensors.BallColor.
     */
    public SlotColorSensors.BallColor toSensorColor(BallColor color) {
        if (color == null) return SlotColorSensors.BallColor.UNKNOWN;
        switch (color) {
            case PURPLE:
                return SlotColorSensors.BallColor.PURPLE;
            case GREEN:
                return SlotColorSensors.BallColor.GREEN;
            case EMPTY:
                return SlotColorSensors.BallColor.NONE;
            default:
                return SlotColorSensors.BallColor.UNKNOWN;
        }
    }

    /**
     * Checks if two states match.
     */
    private boolean colorsMatch(BallColor expected, BallColor actual) {
        if (expected == null || actual == null) return false;
        if (expected == BallColor.UNKNOWN) {
            return actual != BallColor.EMPTY;
        }
        if (actual == BallColor.UNKNOWN) {
            return expected != BallColor.EMPTY;
        }
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
            case PURPLE:
                return "PUR";
            case GREEN:
                return "GRN";
            case UNKNOWN:
                return "UNK";
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
        if (pattern != null) {
            sb.append(", pattern=").append(patternIndex).append("/").append(pattern.length);
        }
        sb.append("}");
        return sb.toString();
    }
}
