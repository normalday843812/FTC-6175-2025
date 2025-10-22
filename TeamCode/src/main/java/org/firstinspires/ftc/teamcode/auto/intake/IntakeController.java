package org.firstinspires.ftc.teamcode.auto.intake;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.AutoSearchConfig.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.auto.inventory.AutoInventory;
import org.firstinspires.ftc.teamcode.auto.motion.FixedFieldHeading;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.auto.vision.BlobDetector;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Intake.AutoMode;
import org.firstinspires.ftc.teamcode.subsystems.IntakeColorSensor;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class IntakeController {
    public enum Result {CONTINUE, TO_SEARCH, TO_DEPOSIT}

    private final MotionController motion;
    private final BlobDetector blobs;
    private final AutoInventory inventory;
    private final Intake intake;
    private final IntakeColorSensor colorSensor;
    private final boolean isRed;
    private final TelemetryHelper tele;

    private final Timer lostTimer = new Timer();

    private boolean targetingPurple = true;

    public IntakeController(MotionController motion,
                            BlobDetector blobs,
                            AutoInventory inventory,
                            Intake intake,
                            IntakeColorSensor colorSensor,
                            boolean isRed,
                            TelemetryHelper tele) {
        this.motion = motion;
        this.blobs = blobs;
        this.inventory = inventory;
        this.intake = intake;
        this.colorSensor = colorSensor;
        this.isRed = isRed;
        this.tele = tele;
        lostTimer.resetTimer();
    }

    public void setTargetColor(boolean purple) {
        this.targetingPurple = purple;
    }

    public Result update(Pose currentPose) {
        // If full, stop intake and deposit
        if (inventory.isFull()) {
            intake.setAutoMode(AutoMode.OFF);
            return Result.TO_DEPOSIT;
        }

        intake.setAutoMode(AutoMode.FORWARD);
        intake.operate();

        BlobDetector.Blob target = targetingPurple ? blobs.bestPurpleBlob() : blobs.bestGreenBlob();
        if (!target.seen) {
            // Fallback: take any if visible
            BlobDetector.Blob any = blobs.pickAccordingToNeed(false);
            if (any.seen) {
                target = any;
            }
        }

        if (target.seen) {
            // Approach
            double vx = clamp(target.cxNorm * BLOB_APPROACH_GAIN, -BLOB_STRAFE_MAX, BLOB_STRAFE_MAX);
            double vy = clamp(target.cyNorm * BLOB_APPROACH_GAIN, -BLOB_FORWARD_MAX, BLOB_FORWARD_MAX);
            HeadingTarget ht = new FixedFieldHeading(isRed ? 0 : 180, "IntakeHeading");
            motion.translateFacing(vx, vy, ht);
            lostTimer.resetTimer();
        } else {
            // No longer seen: check color sensor quickly
            boolean purpleDetected = colorSensor.isPurple();
            boolean greenDetected = colorSensor.isGreen();
            if (purpleDetected || greenDetected) {
                if (purpleDetected) inventory.onCapturedPurple();
                if (greenDetected) inventory.onCapturedGreen();
                intake.setAutoMode(AutoMode.OFF);
                return inventory.isFull() ? Result.TO_DEPOSIT : Result.TO_SEARCH;
            } else {
                // Keep intake a short time to catch rolling piece
                if (lostTimer.getElapsedTimeSeconds() < LOST_BALL_EXTRA_INTAKE_S) {
                    HeadingTarget ht = new FixedFieldHeading(isRed ? 0 : 180, "IntakeHold");
                    motion.translateFacing(0, 0, ht);
                    // Keep intake running
                } else {
                    // Return to search
                    intake.setAutoMode(AutoMode.OFF);
                    return Result.TO_SEARCH;
                }
            }
        }

        addTelemetry(target);
        return Result.CONTINUE;
    }

    private void addTelemetry(BlobDetector.Blob b) {
        tele.addLine("--- INTAKE ---")
                .addData("TargetColor", targetingPurple ? "PURPLE" : "GREEN")
                .addData("BlobSeen", "%b", b.seen)
                .addData("cx, cy", "(%.2f, %.2f)", b.cxNorm, b.cyNorm)
                .addData("Inv", "P:%d G:%d", inventory.purpleCount(), inventory.greenCount());
    }
}