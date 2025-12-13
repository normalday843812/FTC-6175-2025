package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_IDLE_INTAKE_FORWARD;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_IDLE_INTAKE_FORWARD_DURING_ROTATE;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_IDLE_INTAKE_FORWARD_DURING_SHOOTING;
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
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

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
        shooterYaw.start();
        shooter.setAutoRpm(IDLE_RPM);
        transfer.raiseLever();

        inv.reset();
        SpindexerModel model = inv.getModel();
        model.setBucketContents(0, SpindexerModel.BallColor.BALL);
        model.setBucketContents(1, SpindexerModel.BallColor.BALL);
        model.setBucketContents(2, SpindexerModel.BallColor.BALL);

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

        boolean shooterHandledByState = (s == State.PATH_TO_SHOOT || s == State.SHOOTING);
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

    private void syncFrontBucketFromSensor() {
        int frontBucket = spindexer.getCommandedSlot();
        inv.getModel().setBucketAtFront(frontBucket);
        inv.getModel().setBucketContents(
                frontBucket,
                frontHasBall() ? SpindexerModel.BallColor.BALL : SpindexerModel.BallColor.EMPTY
        );
    }

    private void handlePathToShoot() {
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        if (!options.enableDeposit) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.NAVIGATING);
        shooterYaw.lockAllianceGoal();
        shooterYaw.operate();

        if (!pathIssued) {
            double headDeg = getHeadingToGoal(shootPose);
            followToPose(shootPose, headDeg, true);
            pathIssued = true;
        }

        // Spin up shooter while moving
        double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));
        shooter.setAutoRpm(rpm);
        shooter.operate();

        if (!drive.isPathBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_GOAL_S) {
            deposit.reset();
            transitionTo(State.SHOOTING);
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

        if (ui != null) {
            ui.setBase(shooter.isAtTarget(TARGET_RPM_BAND)
                    ? UiLightConfig.UiState.READY
                    : UiLightConfig.UiState.SPINUP);
        }

        shooterYaw.operate();
        double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));

        DepositController.Result r = deposit.update(rpm);
        boolean treatAsShot = (r == DepositController.Result.SHOT)
                || (r == DepositController.Result.FAIL && !frontHasBall());

        if (treatAsShot) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.SHOT, 300);
            // Sync model with current spindexer position before marking shot
            inv.getModel().setBucketAtFront(spindexer.getCommandedSlot());
            inv.onShot();
            shotsRemainingInBatch = Math.max(0, shotsRemainingInBatch - 1);

            if (shotsRemainingInBatch > 0) {
                transitionTo(State.ROTATE_NEXT_BALL);
            } else if (options.enableIntake && inv.setsRemain()) {
                inv.clearBalls();
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                inv.clearBalls();
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
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
        } else if (r == DepositController.Result.FAIL) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.FAIL, 500);
            // Ball still present at front; rotate and try again rather than skipping straight to intake.
            syncFrontBucketFromSensor(); // ensures model knows this front bucket is BALL/EMPTY from sensors
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

        // Deterministic, sensor-driven scan: rotate through slots until slot-0 sensors say a ball is present.
        // This prevents “skip slot 1 then shoot slot 2” when the model gets out of sync.
        if (rotateTargetSlot < 0) {
            int dir = PREFER_CLOCKWISE_ON_TIE ? 1 : -1;
            rotateTargetSlot = ((spindexer.getCommandedSlot() + dir) % SpindexerModel.NUM_BUCKETS + SpindexerModel.NUM_BUCKETS) % SpindexerModel.NUM_BUCKETS;
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
            syncFrontBucketFromSensor(); // mark this bucket BALL in the model
            deposit.reset();
            transitionTo(State.SHOOTING);
            return;
        }

        // No ball at front: rotate to the next slot and try again (up to 3 slots).
        rotateCheckedCount++;
        if (rotateCheckedCount >= SpindexerModel.NUM_BUCKETS) {
            // Could not find a ball at any slot. Treat as empty and proceed to intake/park.
            inv.clearBalls();
            shotsRemainingInBatch = 0;
            if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
            return;
        }

        int dir = PREFER_CLOCKWISE_ON_TIE ? 1 : -1;
        rotateTargetSlot = ((rotateTargetSlot + dir) % SpindexerModel.NUM_BUCKETS + SpindexerModel.NUM_BUCKETS) % SpindexerModel.NUM_BUCKETS;
        spindexer.setSlot(rotateTargetSlot);
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
            followToPose(target, intakeHeadingDeg, false);
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

        // We only have reliable sensing at the front (slot 0).
        if (!spindexer.isSettled()) {
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
            followToPose(creepTarget, Math.toDegrees(current.getHeading()), true);
            pathIssued = true;
        }

        if (transfer != null) transfer.runTransfer(Transfer.CrState.REVERSE);

        intake.setAutoMode(Intake.AutoMode.REVERSE);

        // Only count a new ball when the front was empty and then becomes occupied (edge-based).
        boolean ballNow = frontHasBall();
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
            inv.onBallIntaked();
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
        // Give the ball a moment to fully seat before raising the lever / holding forward.
        if (t.getElapsedTimeSeconds() < Math.max(0.0, INTAKE_POST_DETECT_DWELL_S)) {
            transfer.lowerLever();
            transfer.runTransfer(Transfer.CrState.REVERSE);
            intake.setAutoMode(Intake.AutoMode.OFF);
            return;
        }

        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);
        intake.setAutoMode(Intake.AutoMode.OFF);

        // Wait for ball to be pushed into bucket before deciding next action
        if (t.getElapsedTimeSeconds() < Math.max(0.0, INTAKE_POST_DETECT_DWELL_S) + TELEOP_FEED_DWELL_S) {
            return;
        }

        // Continue intaking at current position if we have empty slots
        if (inv.hasEmptySlots()) {
            transitionTo(State.ALIGN_EMPTY_SLOT);
        } else if (inv.hasBalls() && options.enableDeposit) {
            // Full (per model) - audit to be sure before committing to shooting.
            if (INVENTORY_AUDIT_ENABLED) {
                auditContinueIntakeHere = true;
                transitionTo(State.AUDIT_INVENTORY);
            } else {
                transitionTo(State.PATH_TO_SHOOT);
            }
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
        inv.getModel().setBucketContents(
                frontBucket,
                now ? SpindexerModel.BallColor.BALL : SpindexerModel.BallColor.EMPTY
        );

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
            followToPose(finalPose, headDeg, true);
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
        } else if (newState == State.AUDIT_INVENTORY) {
            auditTargetSlot = 0;
            auditLastReading = null;
            auditStableCount = 0;
            tAudit.resetTimer();
        }
    }

    // --- Path Helpers (using Pedro directly) ---

    private void followToPose(Pose target, double headDeg, boolean enableBezier) {
        Pose current = follower.getPose();
        Pose control = midpointControl(current, target);
        PathChain chain;
        if (enableBezier) {
            chain = follower.pathBuilder()
                    .addPath(new BezierCurve(follower::getPose, control, target))
                    .setLinearHeadingInterpolation(
                            current.getHeading(),
                            Math.toRadians(headDeg),
                            0.8
                    )
                    .build();
        } else {
            chain = follower.pathBuilder()
                    .addPath(new BezierLine(follower::getPose, target))
                    .setLinearHeadingInterpolation(
                            current.getHeading(),
                            Math.toRadians(headDeg),
                            0.8
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
        tele.addLine("=== AUTO ===")
                .addData("State", s::name)
                .addData("t", "%.2f", t.getElapsedTimeSeconds())
                .addData("Balls", "%d", inv.getBallCount())
                .addData("FrontHasBall", () -> frontHasBall())
                .addData("ShotsRemaining", "%d", shotsRemainingInBatch)
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
                .addData("ClearAttempts", "%d", clearAttempts)
                .addData("ClearPhase", "%d", clearPhase)
                .addData("ClearReturnSlot", "%d", clearReturnSlot)
                .addData("AlignChecked", "%d", alignCheckedCount)
                .addData("RotateTarget", "%d", rotateTargetSlot)
                .addData("RotateChecked", "%d", rotateCheckedCount);
    }
}
