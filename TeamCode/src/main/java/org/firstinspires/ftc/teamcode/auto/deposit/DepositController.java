package org.firstinspires.ftc.teamcode.auto.deposit;

import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.BLUE_SHOT_X;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.BLUE_SHOT_Y;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.DRIVE_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.MAX_FEEDS;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.RED_SHOT_X;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.RED_SHOT_Y;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.RPM_RECOVER_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.SHOOT_RPM_BAND;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.SHOOT_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.SPINDEXER_INDEX_TIME_S;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.SPINUP_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoDepositConfig.TOTAL_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoMotionConfig.DRIVE_STOP_DIST_IN;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.geom.FieldBounds;
import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
import org.firstinspires.ftc.teamcode.config.AutoDepositConfig;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

/**
 * Spins up -> Drives -> Waits for shooting <-> Feeds -> done
 */
public class DepositController {
    private enum State {SPINUP, DRIVE, SHOOT_WAIT, FEEDING, DONE}

    private enum FeedPhase {INDEX, WAIT_INDEX, FLICK, WAIT_FLICK, COMPLETE}

    private FeedPhase feedPhase = FeedPhase.COMPLETE;
    private final Timer indexTimer = new Timer();
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final MotionController motion;
    private final HeadingTarget headingTarget;
    private final boolean isRed;
    private final TelemetryHelper tele;

    private final Timer stateTimer = new Timer();
    private final Timer totalTimer = new Timer();
    private final Timer feedTimer = new Timer();

    private int feedsDone = 0;
    private State state = State.SPINUP;
    private boolean feedTriggeredThisCycle = false;

    public DepositController(
            Shooter shooter,
            Spindexer spindexer,
            Transfer transfer,
            MotionController motion,
            HeadingTarget headingTarget,
            boolean isRed,
            TelemetryHelper tele) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.motion = motion;
        this.headingTarget = headingTarget;
        this.isRed = isRed;
        this.tele = tele;

        stateTimer.resetTimer();
        totalTimer.resetTimer();
    }

    /**
     * @return true when deposit sequence finished
     */
    public boolean update() {
        // Keep shooter and transfer targets alive every loop
        shooter.setAutoRpm(SHOOT_TARGET_RPM);
        shooter.operate();
        transfer.operate();

        switch (state) {
            case SPINUP:
                stepSpinup();
                break;
            case DRIVE:
                stepDrive();
                break;
            case SHOOT_WAIT:
                stepShootWait();
                break;
            case FEEDING:
                stepFeeding();
                break;
            case DONE:
                motion.holdHeading(headingTarget);
                break;
        }

        addTelemetry(targetShotPose());
        return state == State.DONE;
    }

    private void stepSpinup() {
        if (stateTimer.getElapsedTimeSeconds() > SPINUP_TIMEOUT_S || shooter.isAtTarget(SHOOT_RPM_BAND)) {
            transition(State.DRIVE);
        } else {
            motion.holdHeading(headingTarget);
        }
    }

    private void stepDrive() {
        Pose target = targetShotPose();
        target = FieldBounds.clampToAllianceRect(target, isRed);

        boolean arrived = motion.driveToPose(target, DRIVE_STOP_DIST_IN, DRIVE_TIMEOUT_S, headingTarget);
        if (arrived || stateTimer.getElapsedTimeSeconds() > DRIVE_TIMEOUT_S) {
            transition(State.SHOOT_WAIT);
        }
    }

    private void stepShootWait() {
        // Completion conditions
        if (feedsDone >= MAX_FEEDS || totalTimer.getElapsedTimeSeconds() > TOTAL_TIMEOUT_S) {
            transition(State.DONE);
            return;
        }

        // If at RPM, feed; else wait until recovery timeout then force a feed
        if (shooter.isAtTarget(SHOOT_RPM_BAND)) {
            triggerFeed();
            transition(State.FEEDING);
            return;
        }

        if (stateTimer.getElapsedTimeSeconds() > RPM_RECOVER_TIMEOUT_S) {
            triggerFeed();
            transition(State.FEEDING);
            return;
        }

        // Otherwise hold heading and keep shooter/hood alive
        motion.holdHeading(headingTarget);
    }

    private void stepFeeding() {
        motion.holdHeading(headingTarget);
        shooter.operate();
        transfer.operate();

        switch (feedPhase) {
            case WAIT_INDEX:
                if (indexTimer.getElapsedTimeSeconds() >= SPINDEXER_INDEX_TIME_S) {
                    if (!AutoDepositConfig.REQUIRE_RPM_AT_FLICK || shooter.isAtTarget(SHOOT_RPM_BAND)) {
                        transfer.flick();
                        feedPhase = FeedPhase.WAIT_FLICK;
                    }
                }
                break;

            case WAIT_FLICK:
                if (transfer.isIdle()) {
                    feedPhase = FeedPhase.COMPLETE;
                    feedsDone++;
                    feedTriggeredThisCycle = false;

                    if (feedsDone >= MAX_FEEDS || totalTimer.getElapsedTimeSeconds() > TOTAL_TIMEOUT_S) {
                        transition(State.DONE);
                    } else {
                        transition(State.SHOOT_WAIT);
                    }
                }
                break;

            default:
                break;
        }
    }

    private void triggerFeed() {
        if (!feedTriggeredThisCycle) {
            spindexer.stepForward();
            indexTimer.resetTimer();
            feedPhase = FeedPhase.WAIT_INDEX;
            feedTimer.resetTimer();
            feedTriggeredThisCycle = true;
        }
    }

    private void transition(State next) {
        state = next;
        stateTimer.resetTimer();
        if (next != State.FEEDING) {
            feedTriggeredThisCycle = false;
        }
    }

    private Pose targetShotPose() {
        Pose want = isRed
                ? new Pose(RED_SHOT_X, RED_SHOT_Y, 0)
                : new Pose(BLUE_SHOT_X, BLUE_SHOT_Y, 0);
        // Clamp shooting pose to alliance rect
        return FieldBounds.clampToAllianceRect(want, isRed);
    }

    public void startCycle() {
        feedsDone = 0;
        feedTriggeredThisCycle = false;
        feedPhase = FeedPhase.COMPLETE;
        state = State.SPINUP;
        stateTimer.resetTimer();
        totalTimer.resetTimer();
    }

    // removed mapHoodForRpm

    private void addTelemetry(Pose target) {
        tele.addLine("--- DEPOSIT ---")
                .addData("State", state::name)
                .addData("Feeds", "%d/%d", feedsDone, MAX_FEEDS)
                .addData("TotalT", "%.2f", totalTimer.getElapsedTimeSeconds())
                .addData("ShotPose", "(%.1f, %.1f)", target.getX(), target.getY());
    }
}
