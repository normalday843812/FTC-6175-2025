package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_IDLE_INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_IDLE_INTAKE_FORWARD_DURING_ROTATE;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_IDLE_INTAKE_FORWARD_DURING_SHOOTING;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_COLOR_ID_ENABLED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_COLOR_ID_SENSOR_SETTLE_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_COLOR_ID_RETRY_MAX;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DEPOSIT_EMPTY_CONFIRM_CYCLES;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_HOLD_TRANSFER_DURING_ROTATE;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_HOLD_TRANSFER_DURING_SHOOTING;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FRONT_CLEAR_EMPTY_CONFIRM_CYCLES;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FRONT_CLEAR_ENABLED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FRONT_CLEAR_FORWARD_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FRONT_CLEAR_MAX_ATTEMPTS;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.FRONT_CLEAR_REVERSE_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INVENTORY_AUDIT_CONFIRM_CYCLES;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INVENTORY_AUDIT_ENABLED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INVENTORY_AUDIT_SENSOR_SETTLE_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INVENTORY_AUDIT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_CREEP_DISTANCE;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_CONFIRM_CYCLES;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_FORWARD_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_POST_DETECT_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_KEEP_SHOOTER_SPUN_UP_BETWEEN_SHOTS;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_SHOOT_EMPTY_PRECHECK_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.REQUIRE_FULL_BEFORE_SHOOT;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.ROTATE_NEXT_BALL_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.ROTATE_NEXT_BALL_SENSOR_SETTLE_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_FEED_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PREFER_CLOCKWISE_ON_TIE;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATH_TIMEOUT_TO_GOAL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATH_TIMEOUT_TO_INTAKE_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.GOAL_BLUE_X;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.GOAL_BLUE_Y;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.GOAL_RED_X;
import static org.firstinspires.ftc.teamcode.config.DecodeGameConfig.GOAL_RED_Y;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.IDLE_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MAX_RPM;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.AutoPathConfig;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.config.UiLightConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class AutoManager {
    public enum State {
        PATH_TO_SHOOT,
        SHOOTING,
        ROTATE_NEXT_BALL,
        PATH_TO_INTAKE,
        ALIGN_EMPTY_SLOT,
        CLEAR_FRONT,
        INTAKING,
        STORE_BALL,
        AUDIT_INVENTORY,
        FINAL_PARK,
        DONE
    }

    public static final class Options {
        public final boolean enableIntake;
        public final boolean enableDeposit;
        public final boolean enableFinalMove;

        public Options(boolean enableIntake, boolean enableDeposit, boolean enableFinalMove) {
            this.enableIntake = enableIntake;
            this.enableDeposit = enableDeposit;
            this.enableFinalMove = enableFinalMove;
        }

        public static Options defaults() {
            return new Options(true, true, true);
        }
    }

    // Dependencies
    private final TelemetryHelper tele;
    private final Mecanum drive;
    private final Follower follower;
    private final Shooter shooter;
    private final ShooterYaw shooterYaw;
    private final Spindexer spindexer;
    private final Intake intake;
    private final SlotColorSensors slots;
    private final InventoryManager inv;
    private final Transfer transfer;
    private final boolean isRed;
    private final double goalX, goalY;
    private final Pose shootPose, finalPose;
    private final DepositController deposit;
    private final UiLight ui;
    private final Options options;

    // State
    private final Timer t = new Timer();
    private final Timer tAudit = new Timer();
    private final Timer tRotate = new Timer();
    private final Timer tAlign = new Timer();
    private final Timer tColorId = new Timer();
    private State s;
    private boolean pathIssued = false;
    private boolean intakeSawEmpty = false;
    private int intakeConfirmCount = 0;
    private int spindexReturnSlot = 0;
    private int clearReturnSlot = 0;
    private int clearAttempts = 0;
    private int clearPhase = 0;
    private int clearEmptyConfirmCount = 0;
    private int alignCheckedCount = 0;
    private int auditTargetSlot = 0;
    private Boolean auditLastReading = null;
    private int auditStableCount = 0;
    private boolean auditContinueIntakeHere = false;
    private int rotateTargetSlot = -1;
    private int rotateCheckedCount = 0;
    private int shotsRemainingInBatch = 0;

    // Color identification (bucket colors) while driving to the goal.
    private boolean colorIdDone = false;
    private int colorIdSlot = 0;
    private int colorIdRetryCount = 0;
    private int colorIdPhase = 0;

    // Intaking: capture a best-effort color at detection time and commit it once the ball is seated.
    private boolean pendingIntakeCommit = false;
    private SlotColorSensors.BallColor pendingIntakeColor = SlotColorSensors.BallColor.UNKNOWN;

    public AutoManager(Mecanum drive,
                       Shooter shooter,
                       ShooterYaw shooterYaw,
                       Spindexer spindexer,
                       Intake intake,
                       Transfer transfer,
                       SlotColorSensors slots,
                       InventoryManager inv,
                       boolean isRed,
                       Pose shootPose,
                       Pose finalPose,
                       UiLight ui,
                       TelemetryHelper tele) {
        this(drive, shooter, shooterYaw, spindexer, intake, transfer, slots, inv,
                isRed, shootPose, finalPose, ui, tele, Options.defaults());
    }

    public AutoManager(Mecanum drive,
                       Shooter shooter,
                       ShooterYaw shooterYaw,
                       Spindexer spindexer,
                       Intake intake,
                       Transfer transfer,
                       SlotColorSensors slots,
                       InventoryManager inv,
                       boolean isRed,
                       Pose shootPose,
                       Pose finalPose,
                       UiLight ui,
                       TelemetryHelper tele,
                       Options options) {
        this.drive = drive;
        this.follower = drive.getFollower();
        this.shooter = shooter;
        this.shooterYaw = shooterYaw;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.intake = intake;
        this.slots = slots;
        this.inv = inv;
        this.isRed = isRed;
        this.goalX = isRed ? GOAL_RED_X : GOAL_BLUE_X;
        this.goalY = isRed ? GOAL_RED_Y : GOAL_BLUE_Y;
        this.shootPose = shootPose;
        this.finalPose = finalPose;
        this.ui = ui;
        this.tele = tele;
        this.deposit = new DepositController(shooter, transfer, spindexer, slots, tele);
        this.options = options == null ? Options.defaults() : options;
    }

    public void start(boolean depositRoute) {
        spindexer.setSlot(0);
        shooter.start();
        // Autonomous does not use ShooterYaw goal tracking; keep it centered.
        shooterYaw.holdCenter();
        shooter.setAutoRpm(IDLE_RPM);
        transfer.raiseLever();

        inv.reset();
        SpindexerModel model = inv.getModel();
        // Assume we start with 3 balls, but the colors are not known until we explicitly identify them.
        model.setBucketContents(0, SpindexerModel.BallColor.UNKNOWN);
        model.setBucketContents(1, SpindexerModel.BallColor.UNKNOWN);
        model.setBucketContents(2, SpindexerModel.BallColor.UNKNOWN);
        resetColorId();
        pendingIntakeCommit = false;
        pendingIntakeColor = SlotColorSensors.BallColor.UNKNOWN;

        if (depositRoute && options.enableDeposit) {
            shotsRemainingInBatch = SpindexerModel.NUM_BUCKETS;
            s = State.PATH_TO_SHOOT;
        } else {
            shotsRemainingInBatch = 0;
            s = options.enableFinalMove ? State.FINAL_PARK : State.DONE;
        }
        pathIssued = false;
        t.resetTimer();
    }

    public void update() {
        if (slots != null) slots.update();
        if (ui != null) ui.update();
        if (intake != null) {
            intake.setAutoMode(AUTO_IDLE_INTAKE_FORWARD ? Intake.AutoMode.FORWARD : Intake.AutoMode.OFF);
        }

        switch (s) {
            case PATH_TO_SHOOT:
                handlePathToShoot();
                break;

            case SHOOTING:
                handleShooting();
                break;

            case ROTATE_NEXT_BALL:
                handleRotateNextBall();
                break;

            case PATH_TO_INTAKE:
                handlePathToIntake();
                break;

            case ALIGN_EMPTY_SLOT:
                handleAlignEmptySlot();
                break;

            case CLEAR_FRONT:
                handleClearFront();
                break;

            case INTAKING:
                handleIntaking();
                break;

            case STORE_BALL:
                handleStoreBall();
                break;

            case AUDIT_INVENTORY:
                handleAuditInventory();
                break;

            case FINAL_PARK:
                handleFinalPark();
                break;

            case DONE:
                handleDone();
                break;
        }

        boolean keepShooterHot = AUTO_KEEP_SHOOTER_SPUN_UP_BETWEEN_SHOTS && shotsRemainingInBatch > 0;
        boolean shooterHandledByState = (s == State.PATH_TO_SHOOT || s == State.SHOOTING)
                || (keepShooterHot && s == State.ROTATE_NEXT_BALL);
        if (!shooterHandledByState) {
            shooter.setAutoRpm(IDLE_RPM);
            shooter.operate();
        }

        if (spindexer != null) spindexer.operate();
        if (transfer != null) transfer.operate();
        if (intake != null) intake.operate();
        addTelemetry();
    }

    private boolean frontHasBall() {
        return slots != null && slots.hasFrontBall();
    }

    private boolean intakeHasBall() {
        return slots != null && slots.hasIntakeBall();
    }

    private boolean anyBallDetected() {
        return frontHasBall() || intakeHasBall();
    }

    private SlotColorSensors.BallColor bestDetectedBallColor() {
        if (slots == null) return SlotColorSensors.BallColor.UNKNOWN;
        if (slots.hasFrontBall()) return slots.getFrontColor();
        if (slots.hasIntakeBall()) return slots.getIntakeColor();
        return SlotColorSensors.BallColor.NONE;
    }

    private void syncFrontBucketFromSensor() {
        int frontBucket = spindexer.getCommandedSlot();
        inv.getModel().setBucketAtFront(frontBucket);
        inv.getModel().setBucketContents(frontBucket, frontBucketColorFromSensors());
    }

    private SpindexerModel.BallColor frontBucketColorFromSensors() {
        if (slots == null) {
            return SpindexerModel.BallColor.UNKNOWN;
        }
        SlotColorSensors.BallColor c = slots.getFrontColor();
        switch (c) {
            case PURPLE:
                return SpindexerModel.BallColor.PURPLE;
            case GREEN:
                return SpindexerModel.BallColor.GREEN;
            case NONE:
                return SpindexerModel.BallColor.EMPTY;
            default:
                return SpindexerModel.BallColor.UNKNOWN;
        }
    }

    private void resetColorId() {
        colorIdDone = !AUTO_COLOR_ID_ENABLED;
        colorIdSlot = 0;
        colorIdRetryCount = 0;
        colorIdPhase = 0;
        tColorId.resetTimer();
    }

    /**
     * Identifies bucket colors by rotating the spindexer and sampling the front color sensors.
     *
     * <p>This is intended to run in parallel while driving to the goal.</p>
     */
    private void updateColorId() {
        if (!AUTO_COLOR_ID_ENABLED || colorIdDone) return;
        if (spindexer == null) return;

        // Phase 0: move/hold at the target slot and wait for settle + sensor dwell.
        if (colorIdPhase == 0) {
            if (spindexer.getCommandedSlot() != colorIdSlot) {
                spindexer.setSlot(colorIdSlot);
            }

            if (!spindexer.isSettled()) {
                tColorId.resetTimer();
                return;
            }

            if (tColorId.getElapsedTimeSeconds() < Math.max(0.0, AUTO_COLOR_ID_SENSOR_SETTLE_S)) {
                return;
            }

            colorIdPhase = 1;
        }

        // Phase 1: read & commit this slot's color.
        if (colorIdPhase == 1) {
            syncFrontBucketFromSensor();
            SpindexerModel.BallColor seen = inv.getModel().getBucketContents(colorIdSlot);

            // If we expected a ball but saw NONE, do a single back/forward reseat before giving up.
            if (seen == SpindexerModel.BallColor.EMPTY && colorIdRetryCount < Math.max(0, AUTO_COLOR_ID_RETRY_MAX)) {
                colorIdRetryCount++;
                int back = (colorIdSlot - 1 + SpindexerModel.NUM_BUCKETS) % SpindexerModel.NUM_BUCKETS;
                spindexer.setSlot(back);
                colorIdPhase = 2;
                tColorId.resetTimer();
                return;
            }

            // Advance to next slot.
            colorIdRetryCount = 0;
            colorIdSlot++;
            if (colorIdSlot >= SpindexerModel.NUM_BUCKETS) {
                colorIdDone = true;
                return;
            }
            spindexer.setSlot(colorIdSlot);
            colorIdPhase = 0;
            tColorId.resetTimer();
            return;
        }

        // Phase 2: waiting at the "back" slot before returning to retry.
        if (colorIdPhase == 2) {
            if (!spindexer.isSettled()) {
                tColorId.resetTimer();
                return;
            }
            if (tColorId.getElapsedTimeSeconds() < Math.max(0.0, AUTO_COLOR_ID_SENSOR_SETTLE_S)) {
                return;
            }
            spindexer.setSlot(colorIdSlot);
            colorIdPhase = 3;
            tColorId.resetTimer();
            return;
        }

        // Phase 3: wait after returning, then re-read.
        if (colorIdPhase == 3) {
            if (!spindexer.isSettled()) {
                tColorId.resetTimer();
                return;
            }
            if (tColorId.getElapsedTimeSeconds() < Math.max(0.0, AUTO_COLOR_ID_SENSOR_SETTLE_S)) {
                return;
            }
            colorIdPhase = 1;
        }
    }

    private void handlePathToShoot() {
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        if (!options.enableDeposit) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        updateColorId();

        if (ui != null) ui.setBase(UiLightConfig.UiState.NAVIGATING);
        shooterYaw.operate();

        if (!pathIssued) {
            double headDeg = getHeadingToGoal(shootPose);
            followToPose(shootPose, headDeg, AutoPathConfig.TO_SHOOT_CONTROL_POINTS, true);
            pathIssued = true;
        }

        // Spin up shooter while moving
        double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));
        shooter.setAutoRpm(rpm);
        shooter.operate();

        if (!drive.isPathBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_GOAL_S) {
            deposit.reset();
            transitionTo(State.ROTATE_NEXT_BALL);
        }
    }

    private void handleShooting() {
        transfer.raiseLever();
        transfer.runTransfer(AUTO_HOLD_TRANSFER_DURING_SHOOTING ? Transfer.CrState.FORWARD : Transfer.CrState.OFF);
        intake.setAutoMode(AUTO_IDLE_INTAKE_FORWARD_DURING_SHOOTING ? Intake.AutoMode.FORWARD : Intake.AutoMode.OFF);
        if (!options.enableDeposit) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        // If we just entered shooting but slot-0 is empty, don't waste time spinning up just to hit NO_BALL.
        if (shotsRemainingInBatch > 0
                && t.getElapsedTimeSeconds() < Math.max(0.0, AUTO_SHOOT_EMPTY_PRECHECK_S)
                && !frontHasBall()) {
            transitionTo(State.ROTATE_NEXT_BALL);
            return;
        }

        if (ui != null) {
            ui.setBase(shooter.isAtTarget(TARGET_RPM_BAND)
                    ? UiLightConfig.UiState.READY
                    : UiLightConfig.UiState.SPINUP);
        }

        shooterYaw.operate();
        double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));

        DepositController.Result r = deposit.update(rpm);
        // If the deposit routine can't reconcile sensors vs. RPM-drop after REFIRE_MAX attempts, assume the shot
        // succeeded anyway (per plan) and advance the inventory to avoid getting stuck.
        boolean treatAsShot = (r == DepositController.Result.SHOT) || (r == DepositController.Result.FAIL);

        if (treatAsShot) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.SHOT, 300);
            // Sync model with current spindexer position before marking shot
            inv.getModel().setBucketAtFront(spindexer.getCommandedSlot());
            inv.onShot();
            shotsRemainingInBatch = Math.max(0, shotsRemainingInBatch - 1);

            if (shotsRemainingInBatch > 0) {
                transitionTo(State.ROTATE_NEXT_BALL);
            } else {
                // Batch complete. If ANY sensor still sees a ball, keep shooting until it's gone.
                if (anyBallDetected()) {
                    shotsRemainingInBatch = 1;
                    transitionTo(State.ROTATE_NEXT_BALL);
                    return;
                }

                if (options.enableIntake && inv.setsRemain()) {
                    inv.clearBalls();
                    transitionTo(State.PATH_TO_INTAKE);
                } else {
                    inv.clearBalls();
                    transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
                }
            }
        } else if (r == DepositController.Result.NO_BALL) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.FAIL, 300);
            // Don't mark buckets empty on NO_BALL; just rotate and search for the next loaded ball.
            if (shotsRemainingInBatch > 0) {
                transitionTo(State.ROTATE_NEXT_BALL);
            } else if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        }
    }

    private void handleRotateNextBall() {
        transfer.raiseLever();
        transfer.runTransfer(AUTO_HOLD_TRANSFER_DURING_ROTATE ? Transfer.CrState.FORWARD : Transfer.CrState.OFF);
        intake.setAutoMode(AUTO_IDLE_INTAKE_FORWARD_DURING_ROTATE ? Intake.AutoMode.FORWARD : Intake.AutoMode.OFF);
        if (AUTO_KEEP_SHOOTER_SPUN_UP_BETWEEN_SHOTS && shotsRemainingInBatch > 0) {
            double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));
            shooter.setAutoRpm(rpm);
            shooter.operate();
        }

        // Avoid getting stuck forever if spindexer never settles / keeps getting spammed with commands.
        if (t.getElapsedTimeSeconds() >= Math.max(0.0, ROTATE_NEXT_BALL_TIMEOUT_S)) {
            if (INVENTORY_AUDIT_ENABLED) {
                auditContinueIntakeHere = false;
                transitionTo(State.AUDIT_INVENTORY);
            } else if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        // If we have a pattern but some bucket colors are still unknown, prioritize finishing
        // the color-ID scan before selecting a target bucket to shoot.
        updateColorId();
        if (AUTO_COLOR_ID_ENABLED
                && inv.getModel().isPatternKnown()
                && inv.getModel().hasUnknownBalls()
                && !colorIdDone) {
            return;
        }

        // Select the next bucket to shoot.
        // - If a pattern is known and bucket colors are known, pick the next pattern color.
        // - Otherwise, fall back to "shoot any loaded ball".
        if (rotateTargetSlot < 0) {
            rotateTargetSlot = inv.decideTargetSlot(spindexer);
            if (rotateTargetSlot < 0) {
                // No balls left to shoot.
                inv.clearBalls();
                shotsRemainingInBatch = 0;
                if (options.enableIntake && inv.setsRemain()) {
                    transitionTo(State.PATH_TO_INTAKE);
                } else {
                    transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
                }
                return;
            }
        }

        // Only issue the command once per target (don't compare to getCurrentSlot(), which depends on STEP tuning).
        if (spindexer.getCommandedSlot() != rotateTargetSlot) {
            spindexer.setSlot(rotateTargetSlot);
        }

        if (!spindexer.isSettled()) {
            tRotate.resetTimer();
            return;
        }

        // Give the transfer a moment to seat the ball after rotation before trusting the sensor.
        if (tRotate.getElapsedTimeSeconds() < Math.max(0.0, ROTATE_NEXT_BALL_SENSOR_SETTLE_S)) {
            return;
        }

        if (frontHasBall()) {
            syncFrontBucketFromSensor(); // commit this bucket's color to the model
            deposit.reset();
            transitionTo(State.SHOOTING);
            return;
        }

        // Target bucket did not actually have a ball at the front. Accept sensor truth and try again.
        syncFrontBucketFromSensor();
        rotateCheckedCount++;
        if (rotateCheckedCount >= SpindexerModel.NUM_BUCKETS) {
            inv.clearBalls();
            shotsRemainingInBatch = 0;
            if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        rotateTargetSlot = -1;
        tRotate.resetTimer();
    }

    private void handlePathToIntake() {
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        if (!options.enableIntake) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);

        if (!pathIssued) {
            Pose target = inv.nextIntakePose(isRed);
            double intakeHeadingDeg = Math.toDegrees(target.getHeading());
            followToPose(target, intakeHeadingDeg, AutoPathConfig.TO_INTAKE_CONTROL_POINTS, false);
            pathIssued = true;
        }

        if (!drive.isPathBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_INTAKE_S) {
            inv.markOneIntakeSetVisited();
            transitionTo(State.ALIGN_EMPTY_SLOT);
        }
    }

    private void handleAlignEmptySlot() {
        // Keep balls retained while we rotate the spindexer to find an empty front.
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);

        // Safety net: if the model believes we are already full, don't keep scanning for an "empty" slot.
        // Audit once to confirm, then proceed to shooting (or continue intake) based on audited truth.
        if (inv.getBallCount() >= SpindexerModel.NUM_BUCKETS) {
            if (INVENTORY_AUDIT_ENABLED) {
                auditContinueIntakeHere = true;
                transitionTo(State.AUDIT_INVENTORY);
                return;
            }
            if (options.enableDeposit) {
                shotsRemainingInBatch = SpindexerModel.NUM_BUCKETS;
                resetColorId();
                transitionTo(State.PATH_TO_SHOOT);
                return;
            }
        }

        // We only have reliable sensing at the front (slot 0).
        if (!spindexer.isSettled()) {
            tAlign.resetTimer();
            return;
        }

        // Give sensors time to settle after rotation so we don't "skip past" an empty slot.
        if (tAlign.getElapsedTimeSeconds() < Math.max(0.0, ROTATE_NEXT_BALL_SENSOR_SETTLE_S)) {
            return;
        }

        syncFrontBucketFromSensor();
        boolean frontBall = frontHasBall();

        // Front is aligned to an empty bucket: safe to intake.
        if (!frontBall) {
            transitionTo(State.INTAKING);
            return;
        }

        // Sensor-driven scan for an empty slot: rotate through up to 3 buckets until front clears.
        alignCheckedCount++;
        if (alignCheckedCount >= SpindexerModel.NUM_BUCKETS) {
            // Couldn't find an empty front after scanning all buckets.
            // Try a mechanical clear cycle (pull ball back / re-seat), then audit/bail if needed.
            startClearFront();
            return;
        }

        if (PREFER_CLOCKWISE_ON_TIE) {
            spindexer.stepForward();
        } else {
            spindexer.stepBackward();
        }
        tAlign.resetTimer();
    }

    private void startClearFront() {
        if (!FRONT_CLEAR_ENABLED || FRONT_CLEAR_MAX_ATTEMPTS <= 0) {
            if (INVENTORY_AUDIT_ENABLED) {
                auditContinueIntakeHere = false;
                transitionTo(State.AUDIT_INVENTORY);
            } else if (inv.hasBalls() && options.enableDeposit) {
                transitionTo(State.PATH_TO_SHOOT);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        if (clearAttempts >= FRONT_CLEAR_MAX_ATTEMPTS) {
            if (INVENTORY_AUDIT_ENABLED) {
                auditContinueIntakeHere = false;
                transitionTo(State.AUDIT_INVENTORY);
            } else if (inv.hasBalls() && options.enableDeposit) {
                transitionTo(State.PATH_TO_SHOOT);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        clearAttempts++;
        clearReturnSlot = ((spindexReturnSlot % SpindexerModel.NUM_BUCKETS) + SpindexerModel.NUM_BUCKETS) % SpindexerModel.NUM_BUCKETS;
        transitionTo(State.CLEAR_FRONT);
    }

    private void handleClearFront() {
        // Phase 0: return spindexer to the slot we were on before the failed align attempt.
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intake.setAutoMode(Intake.AutoMode.OFF);

        int target = ((clearReturnSlot % SpindexerModel.NUM_BUCKETS) + SpindexerModel.NUM_BUCKETS) % SpindexerModel.NUM_BUCKETS;
        if (spindexer.getCommandedSlot() != target) {
            spindexer.setSlot(target);
            return;
        }

        if (!spindexer.isSettled()) {
            return;
        }

        if (clearPhase == 0) {
            clearPhase = 1;
            clearEmptyConfirmCount = 0;
            t.resetTimer();
            return;
        }

        if (clearPhase == 1) {
            // Phase 1: lever down, transfer reverse to pull the ball back / re-seat it.
            transfer.lowerLever();
            transfer.runTransfer(Transfer.CrState.REVERSE);

            boolean empty = !frontHasBall();
            if (empty) {
                clearEmptyConfirmCount++;
            } else {
                clearEmptyConfirmCount = 0;
            }

            if (clearEmptyConfirmCount >= Math.max(1, FRONT_CLEAR_EMPTY_CONFIRM_CYCLES)) {
                syncFrontBucketFromSensor(); // mark the front bucket EMPTY before starting intake
                transitionTo(State.INTAKING);
                return;
            }

            if (t.getElapsedTimeSeconds() >= Math.max(0.0, FRONT_CLEAR_REVERSE_DWELL_S)) {
                clearPhase = 2;
                clearEmptyConfirmCount = 0;
                t.resetTimer();
            }
            return;
        }

        if (clearPhase == 2) {
            // Phase 2: lever up, transfer forward to hold, then retry spindexing.
            transfer.raiseLever();
            transfer.runTransfer(Transfer.CrState.FORWARD);

            if (t.getElapsedTimeSeconds() >= Math.max(0.0, FRONT_CLEAR_FORWARD_DWELL_S)) {
                clearPhase = 3;
                clearEmptyConfirmCount = 0;
                t.resetTimer();
            }
            return;
        }

        // Phase 3: if still seeing a ball at the front, retry align. If cleared, start intake.
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);

        boolean empty = !frontHasBall();
        if (empty) {
            clearEmptyConfirmCount++;
        } else {
            clearEmptyConfirmCount = 0;
        }

        if (clearEmptyConfirmCount >= Math.max(1, FRONT_CLEAR_EMPTY_CONFIRM_CYCLES)) {
            syncFrontBucketFromSensor();
            transitionTo(State.INTAKING);
            return;
        }

        transitionTo(State.ALIGN_EMPTY_SLOT);
    }

    private void handleIntaking() {
        if (transfer != null) transfer.lowerLever();
        if (!options.enableIntake) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);
        if (transfer != null) transfer.runTransfer(Transfer.CrState.REVERSE);

        // Start path to creep forward on first entry
        if (!pathIssued) {
            Pose current = follower.getPose();
            // Creep in X direction: red = +X, blue = -X
            double targetX = current.getX() + (isRed ? INTAKE_CREEP_DISTANCE : -INTAKE_CREEP_DISTANCE);
            Pose creepTarget = new Pose(targetX, current.getY(), current.getHeading());
            followToPose(creepTarget, Math.toDegrees(current.getHeading()), null, false);
            pathIssued = true;
        }

        if (transfer != null) transfer.runTransfer(Transfer.CrState.REVERSE);

        intake.setAutoMode(Intake.AutoMode.REVERSE);

        // Only count a new ball when the sensors were empty and then become occupied (edge-based).
        boolean ballNow = anyBallDetected();
        if (!ballNow) {
            intakeSawEmpty = true;
            intakeConfirmCount = 0;
        } else if (intakeSawEmpty) {
            intakeConfirmCount++;
        }

        boolean gotBall = intakeSawEmpty && ballNow && intakeConfirmCount >= INTAKE_CONFIRM_CYCLES;
        boolean timeout = t.getElapsedTimeSeconds() >= INTAKE_FORWARD_TIMEOUT_S;

        if (gotBall) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.PICKUP, 250);
            intake.setAutoMode(Intake.AutoMode.OFF);
            inv.getModel().setBucketAtFront(spindexer.getCommandedSlot());
            if (transfer != null) {
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.OFF);
            }
            pendingIntakeCommit = true;
            pendingIntakeColor = (slots != null) ? bestDetectedBallColor() : SlotColorSensors.BallColor.UNKNOWN;
            transitionTo(State.STORE_BALL);
        } else if (timeout) {
            intake.setAutoMode(Intake.AutoMode.OFF);

            // "Finished intaking" at this set (timeout). Audit inventory before deciding next action.
            if (INVENTORY_AUDIT_ENABLED) {
                auditContinueIntakeHere = false;
                transitionTo(State.AUDIT_INVENTORY);
            } else if (inv.hasEmptySlots() && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else if (inv.hasBalls() && options.enableDeposit) {
                transitionTo(State.PATH_TO_SHOOT);
            } else if (inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        }
    }

    private void handleStoreBall() {
        intake.setAutoMode(Intake.AutoMode.OFF);

        // Phase 0: stop everything, raise lever to retain, and allow the ball to settle.
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.OFF);
        if (t.getElapsedTimeSeconds() < Math.max(0.0, INTAKE_POST_DETECT_DWELL_S)) {
            return;
        }

        // Phase 1: feed the ball into the bucket for a fixed dwell, then stop the CR servo.
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        if (t.getElapsedTimeSeconds() < Math.max(0.0, INTAKE_POST_DETECT_DWELL_S) + Math.max(0.0, TELEOP_FEED_DWELL_S)) {
            return;
        }

        // Commit the newly intaked ball to the inventory model once we've finished feeding/seating it.
        if (pendingIntakeCommit) {
            inv.getModel().setBucketAtFront(spindexer.getCommandedSlot());
            inv.onBallIntaked(pendingIntakeColor);
            pendingIntakeCommit = false;
            pendingIntakeColor = SlotColorSensors.BallColor.UNKNOWN;
        }

        transfer.runTransfer(Transfer.CrState.OFF);

        // Continue intaking at current position if we have empty slots
        if (inv.hasEmptySlots()) {
            transitionTo(State.ALIGN_EMPTY_SLOT);
        } else if (inv.hasBalls() && options.enableDeposit) {
            // Full: immediately go shoot. Auditing here can false-negative and cause pointless extra intake cycles.
            shotsRemainingInBatch = SpindexerModel.NUM_BUCKETS;
            resetColorId();
            transitionTo(State.PATH_TO_SHOOT);
        } else {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
        }
    }

    private void handleAuditInventory() {
        if (t.getElapsedTimeSeconds() >= Math.max(0.0, INVENTORY_AUDIT_TIMEOUT_S)) {
            // Don't get stuck auditing forever; treat as "not full" and keep searching.
            if (options.enableIntake) {
                transitionTo(auditContinueIntakeHere ? State.ALIGN_EMPTY_SLOT : State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        // Inventory audit: rotate through slots 0/1/2 and use the slot-0 sensor to rebuild the model.
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intake.setAutoMode(Intake.AutoMode.OFF);

        int target = ((auditTargetSlot % SpindexerModel.NUM_BUCKETS) + SpindexerModel.NUM_BUCKETS) % SpindexerModel.NUM_BUCKETS;
        if (spindexer.getCommandedSlot() != target) {
            spindexer.setSlot(target);
            auditLastReading = null;
            auditStableCount = 0;
            tAudit.resetTimer();
            return;
        }

        if (!spindexer.isSettled()) {
            auditLastReading = null;
            auditStableCount = 0;
            tAudit.resetTimer();
            return;
        }

        if (tAudit.getElapsedTimeSeconds() < INVENTORY_AUDIT_SENSOR_SETTLE_S) {
            return;
        }

        boolean now = frontHasBall();
        if (auditLastReading == null || auditLastReading != now) {
            auditLastReading = now;
            auditStableCount = 1;
        } else {
            auditStableCount++;
        }

        if (auditStableCount < Math.max(1, INVENTORY_AUDIT_CONFIRM_CYCLES)) {
            return;
        }

        // Commit this slot's contents to the model.
        int frontBucket = spindexer.getCommandedSlot();
        inv.getModel().setBucketAtFront(frontBucket);
        inv.getModel().setBucketContents(frontBucket, frontBucketColorFromSensors());

        auditTargetSlot++;
        auditLastReading = null;
        auditStableCount = 0;
        tAudit.resetTimer();

        if (auditTargetSlot < SpindexerModel.NUM_BUCKETS) {
            return;
        }

        // Audit complete: decide where to go.
        boolean full = inv.getBallCount() >= SpindexerModel.NUM_BUCKETS;
        if (full && options.enableDeposit) {
            shotsRemainingInBatch = SpindexerModel.NUM_BUCKETS;
            resetColorId();
            transitionTo(State.PATH_TO_SHOOT);
            return;
        }

        if (REQUIRE_FULL_BEFORE_SHOOT) {
            // Enforce "sets of three": never go shoot unless the audit says we have all 3.
            if (options.enableIntake) {
                transitionTo(auditContinueIntakeHere ? State.ALIGN_EMPTY_SLOT : State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        if (inv.hasBalls() && options.enableDeposit) {
            transitionTo(State.PATH_TO_SHOOT);
        } else if (inv.setsRemain()) {
            transitionTo(State.PATH_TO_INTAKE);
        } else {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
        }
    }

    private void handleFinalPark() {
        transfer.raiseLever();
        if (!options.enableFinalMove) {
            transitionTo(State.DONE);
            return;
        }
        transfer.runTransfer(Transfer.CrState.OFF);
        intake.setAutoMode(Intake.AutoMode.OFF);

        if (ui != null) ui.setBase(UiLightConfig.UiState.PARK);

        if (!pathIssued) {
            double headDeg = Math.toDegrees(finalPose.getHeading());
            followToPose(finalPose, headDeg, AutoPathConfig.TO_FINAL_CONTROL_POINTS, true);
            pathIssued = true;
        }

        if (!drive.isPathBusy() || t.getElapsedTimeSeconds() >= DEFAULT_TIMEOUT_S) {
            transitionTo(State.DONE);
        }
    }

    private void handleDone() {
        transfer.raiseLever();
        if (ui != null) ui.setBase(UiLightConfig.UiState.DONE);
        transfer.runTransfer(Transfer.CrState.OFF);
        drive.setAutoDrive(0, 0, 0, true, 0);
        intake.setAutoMode(Intake.AutoMode.OFF);
        shooter.setAutoRpm(0);
        shooter.operate();
    }

    private void transitionTo(State newState) {
        State oldState = s;
        drive.clearAutoCommand();
        if (follower != null) {
            // Zero any lingering manual-drive vectors before switching back to teleop-drive mode.
            follower.setTeleOpDrive(0, 0, 0);
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0, 0, 0);
        }

        s = newState;
        pathIssued = false;
        t.resetTimer();

        if (newState == State.ALIGN_EMPTY_SLOT) {
            if (oldState != State.CLEAR_FRONT) {
                clearAttempts = 0;
            }
            if (spindexer != null) {
                spindexReturnSlot = spindexer.getCurrentSlot();
            }
            alignCheckedCount = 0;
            tAlign.resetTimer();
        } else if (newState == State.ROTATE_NEXT_BALL) {
            rotateTargetSlot = -1;
            rotateCheckedCount = 0;
            tRotate.resetTimer();
        } else if (newState == State.CLEAR_FRONT) {
            clearPhase = 0;
            clearEmptyConfirmCount = 0;
        } else if (newState == State.INTAKING) {
            intakeSawEmpty = false;
            intakeConfirmCount = 0;
            clearAttempts = 0;
            pendingIntakeCommit = false;
            pendingIntakeColor = SlotColorSensors.BallColor.UNKNOWN;
        } else if (newState == State.AUDIT_INVENTORY) {
            auditTargetSlot = 0;
            auditLastReading = null;
            auditStableCount = 0;
            tAudit.resetTimer();
        }
    }

    // --- Path Helpers (using Pedro directly) ---

    private void followToPose(Pose target, double headDeg, double[][] controlPoints, boolean enableBezier) {
        Pose current = follower.getPose();
        PathChain chain;

        if (enableBezier) {
            java.util.List<Pose> points = new java.util.ArrayList<>();
            points.add(current);

            if (controlPoints != null && controlPoints.length > 0) {
                for (double[] cp : controlPoints) {
                    double headingRad = cp.length > 2 ? Math.toRadians(cp[2]) : target.getHeading();
                    points.add(new Pose(cp[0], cp[1], headingRad));
                }
            } else {
                points.add(midpointControl(current, target));
            }

            points.add(target);

            chain = follower.pathBuilder()
                    .addPath(new BezierCurve(points))
                    .setLinearHeadingInterpolation(
                            current.getHeading(),
                            Math.toRadians(headDeg),
                            AutoPathConfig.HEADING_INTERPOLATION_END_T
                    )
                    .build();
        } else {
            chain = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, target))
                    .setLinearHeadingInterpolation(
                            current.getHeading(),
                            Math.toRadians(headDeg),
                            AutoPathConfig.HEADING_INTERPOLATION_END_T
                    )
                    .build();
        }

        drive.followPath(chain);
    }

    private static Pose midpointControl(Pose start, Pose target) {
        double midX = (start.getX() + target.getX()) / 2.0;
        double midY = (start.getY() + target.getY()) / 2.0;
        return new Pose(midX, midY, target.getHeading());
    }

    private double getHeadingToGoal(Pose fromPose) {
        double dx = goalX - fromPose.getX();
        double dy = goalY - fromPose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    private void addTelemetry() {
        SpindexerModel model = inv.getModel();
        SpindexerModel.BallColor next = model.getNextPatternColor();

        tele.addLine("=== AUTO ===")
                .addData("State", s::name)
                .addData("t", "%.2f", t.getElapsedTimeSeconds())
                .addData("Balls", "%d", inv.getBallCount())
                .addData("FrontHasBall", this::frontHasBall)
                .addData("ShotsRemaining", "%d", shotsRemainingInBatch)
                .addData("Pattern", "%s", model.isPatternKnown() ? "YES" : "no")
                .addData("PatternTag", "%s", model.getPatternTagId() < 0 ? "---" : (DecodeGameConfig.patternNameForTag(model.getPatternTagId()) + " (" + model.getPatternTagId() + ")"))
                .addData("PatternConf", "%.2f", model.getPatternConfidence())
                .addData("PatternIdx", "%d", model.getPatternIndex())
                .addData("NextColor", "%s", next == null ? "---" : next.name())
                .addData("ShootPrecheckS", "%.2f", AUTO_SHOOT_EMPTY_PRECHECK_S)
                .addData("KeepShooterHot", "%b", AUTO_KEEP_SHOOTER_SPUN_UP_BETWEEN_SHOTS)
                .addData("IntakeConfirmCycles", "%d", INTAKE_CONFIRM_CYCLES)
                .addData("PostDetectDwellS", "%.2f", INTAKE_POST_DETECT_DWELL_S)
                .addData("IdleIntakeForward", "%b", AUTO_IDLE_INTAKE_FORWARD)
                .addData("DepositEmptyConfirmCycles", "%d", DEPOSIT_EMPTY_CONFIRM_CYCLES)
                .addData("RequireFullBeforeShoot", "%b", REQUIRE_FULL_BEFORE_SHOOT)
                .addData("AuditEnabled", "%b", INVENTORY_AUDIT_ENABLED)
                .addData("AuditSlot", "%d", auditTargetSlot)
                .addData("AuditTimeoutS", "%.2f", INVENTORY_AUDIT_TIMEOUT_S)
                .addData("RotateSettleS", "%.2f", ROTATE_NEXT_BALL_SENSOR_SETTLE_S)
                .addData("RotateTimeoutS", "%.2f", ROTATE_NEXT_BALL_TIMEOUT_S)
                .addData("ColorIdDone", "%b", colorIdDone)
                .addData("ColorIdSlot", "%d", colorIdSlot)
                .addData("ColorIdPhase", "%d", colorIdPhase)
                .addData("ColorIdRetry", "%d", colorIdRetryCount)
                .addData("ClearAttempts", "%d", clearAttempts)
                .addData("ClearPhase", "%d", clearPhase)
                .addData("ClearReturnSlot", "%d", clearReturnSlot)
                .addData("AlignChecked", "%d", alignCheckedCount)
                .addData("RotateTarget", "%d", rotateTargetSlot)
                .addData("RotateChecked", "%d", rotateCheckedCount);

        tele.addLine("--- BUCKETS ---")
                .addData("B0", model.getBucketContents(0)::name)
                .addData("B1", model.getBucketContents(1)::name)
                .addData("B2", model.getBucketContents(2)::name)
                .addData("AtFront", "%d", model.getBucketAtFront());

        tele.addLine("--- RAMP ---")
                .addData("Placed", "%d", model.getPatternIndex())
                .addData("Shots", () -> rampShotSummary(model));
    }

    private static String rampShotSummary(SpindexerModel model) {
        if (model == null) return "---";
        int placed = Math.min(Math.max(0, model.getPatternIndex()), SpindexerModel.RAMP_INDICES);
        if (placed <= 0) return "---";

        SpindexerModel.BallColor[] history = model.getShotHistory();
        SpindexerModel.BallColor[] pattern = model.getPattern();

        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < placed; i++) {
            if (i > 0) sb.append("  ");
            SpindexerModel.BallColor actual = (history != null && i < history.length) ? history[i] : null;
            SpindexerModel.BallColor expected = (pattern != null && i < pattern.length) ? pattern[i] : null;

            sb.append(i + 1).append(":").append(shortColor(actual));
            if (expected != null) {
                sb.append(actual == expected ? "=" : "!=").append(shortColor(expected));
            }
        }
        return sb.toString();
    }

    private static String shortColor(SpindexerModel.BallColor color) {
        if (color == null) return "?";
        switch (color) {
            case PURPLE:
                return "P";
            case GREEN:
                return "G";
            case UNKNOWN:
                return "U";
            case EMPTY:
                return "-";
        }
        return "?";
    }
}
