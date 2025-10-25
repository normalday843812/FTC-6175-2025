package org.firstinspires.ftc.teamcode.auto.search;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.BLUE_HALF_CENTER_X;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.BLUE_HALF_CENTER_Y;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.FORWARD_SCAN_TIME_S;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.OVERRIDE_TIME_S;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.RED_HALF_CENTER_X;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.RED_HALF_CENTER_Y;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.ROTATE_SCAN_SPEED;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.ROTATE_SCAN_TIME_S;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.SEARCH_BUDGET_S;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.SPIN_IN_PLACE_SPEED;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.SPIN_IN_PLACE_TIME_S;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.auto.inventory.AutoInventory;
import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.auto.vision.BlobDetector;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class SearchController {

    public enum Result {CONTINUE, TO_DEPOSIT, TO_INTAKE}

    private enum ScanState {ROTATE_SCAN, FORWARD_SCAN, SPIN_IN_PLACE}

    private final MotionController motion;
    private final BlobDetector blobs;
    private final AutoInventory inventory;
    private final boolean isRed;
    private final TelemetryHelper tele;

    private final Timer stateTimer = new Timer();
    private final Timer budgetTimer = new Timer();

    private ScanState scan = ScanState.ROTATE_SCAN;

    private BlobDetector.Blob lastChosenBlob = BlobDetector.Blob.none();
    private boolean chosenIsPurple = false;

    public SearchController(MotionController motion,
                            BlobDetector blobs,
                            AutoInventory inventory,
                            boolean isRed,
                            TelemetryHelper tele) {
        this.motion = motion;
        this.blobs = blobs;
        this.inventory = inventory;
        this.isRed = isRed;
        this.tele = tele;
        stateTimer.resetTimer();
        budgetTimer.resetTimer();
    }

    /** @noinspection DataFlowIssue*/
    public Result update(Pose currentPose) {
        if (inventory.isFull()) return Result.TO_DEPOSIT;

        // Preference logic: 2 purple + 1 green target
        AutoInventory.PickupPreference pref = inventory.getPickupPreference();
        boolean allowAny = budgetTimer.getElapsedTimeSeconds() > OVERRIDE_TIME_S;

        BlobDetector.Blob pb = blobs.bestPurpleBlob();
        BlobDetector.Blob gb = blobs.bestGreenBlob();

        BlobDetector.Blob chosen = BlobDetector.Blob.none();
        boolean isPurple = false;

        if (allowAny || pref == AutoInventory.PickupPreference.NONE) {
            // Choose largest visible blob
            if (pb.seen && gb.seen) {
                chosen = (pb.area >= gb.area) ? pb : gb;
                isPurple = (chosen == pb);
            } else if (pb.seen) {
                chosen = pb;
                isPurple = true;
            } else if (gb.seen) {
                chosen = gb;
                isPurple = false;
            }
        } else if (pref == AutoInventory.PickupPreference.PREFER_PURPLE) {
            if (pb.seen) {
                chosen = pb;
                isPurple = true;
            } else if (gb.seen) {
                chosen = gb;
                isPurple = false; // fallback
            }
        } else if (pref == AutoInventory.PickupPreference.PREFER_GREEN) {
            if (gb.seen) {
                chosen = gb;
                isPurple = false;
            } else if (pb.seen) {
                chosen = pb;
                isPurple = true; // fallback
            }
        }

        if (chosen.seen) {
            lastChosenBlob = chosen;
            chosenIsPurple = isPurple;
            return Result.TO_INTAKE;
        }

        // Budget exceeded -> deposit
        if (budgetTimer.getElapsedTimeSeconds() > SEARCH_BUDGET_S) {
            return Result.TO_DEPOSIT;
        }

        // Scanning pattern
        switch (scan) {
            case ROTATE_SCAN: {
                motion.spinInPlace(ROTATE_SCAN_SPEED);

                if (stateTimer.getElapsedTimeSeconds() > ROTATE_SCAN_TIME_S) {
                    scan = ScanState.FORWARD_SCAN;
                    stateTimer.resetTimer();
                }
                break;
            }
            case FORWARD_SCAN: {
                double cx = isRed ? RED_HALF_CENTER_X : BLUE_HALF_CENTER_X;
                double cy = isRed ? RED_HALF_CENTER_Y : BLUE_HALF_CENTER_Y;

                double dx = cx - currentPose.getX();
                double dy = cy - currentPose.getY();

                double vx = clamp(dx * 0.02, -0.25, 0.25);
                double vy = clamp(dy * 0.02, -0.25, 0.25);

                HeadingTarget ht = new FixedFieldHeading(isRed ? 0 : 180, "FaceHalfCenter");
                motion.translateFacing(vx, vy, ht);

                if (stateTimer.getElapsedTimeSeconds() > FORWARD_SCAN_TIME_S) {
                    scan = ScanState.SPIN_IN_PLACE;
                    stateTimer.resetTimer();
                }
                break;
            }
            case SPIN_IN_PLACE: {
                // Slow in-place spin to discover blobs near the robot
                motion.spinInPlace(SPIN_IN_PLACE_SPEED);

                if (stateTimer.getElapsedTimeSeconds() > SPIN_IN_PLACE_TIME_S) {
                    scan = ScanState.ROTATE_SCAN;
                    stateTimer.resetTimer();
                }
                break;
            }
        }

        addTelemetry(pref);
        return Result.CONTINUE;
    }

    public BlobDetector.Blob getLastChosenBlob() {
        return lastChosenBlob;
    }

    public boolean lastChosenIsPurple() {
        return chosenIsPurple;
    }

    public void resetBudget() {
        budgetTimer.resetTimer();
        stateTimer.resetTimer();
        scan = ScanState.ROTATE_SCAN;
    }

    private void addTelemetry(AutoInventory.PickupPreference pref) {
        tele.addLine("--- SEARCH ---")
                .addData("State", scan::name)
                .addData("Budget", "%.1f/%.1f", budgetTimer.getElapsedTimeSeconds(), SEARCH_BUDGET_S)
                .addData("Preference", pref::name);
    }
}
