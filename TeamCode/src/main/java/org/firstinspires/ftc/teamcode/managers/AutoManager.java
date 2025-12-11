package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_FORWARD_SPEED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_FORWARD_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATH_TIMEOUT_TO_GOAL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATH_TIMEOUT_TO_INTAKE_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.IDLE_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MAX_RPM;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.auto.motion.HeadingTarget;
import org.firstinspires.ftc.teamcode.auto.motion.MotionController;
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
        INTAKING,
        STORE_BALL,
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
    private final MotionController motion;
    private final Shooter shooter;
    private final ShooterYaw shooterYaw;
    private final Spindexer spindexer;
    private final Intake intake;
    private final SlotColorSensors slots;
    private final InventoryManager inv;
    private final Transfer transfer;
    private final HeadingTarget heading;
    private final boolean isRed;
    private final Pose shootPose, finalPose;
    private final DepositController deposit;
    private final UiLight ui;
    private final Options options;

    // State
    private final Timer t = new Timer();
    private State s;
    private int shotsRemaining = 3;  // Assume 3 balls at start
    private boolean pathIssued = false;

    public AutoManager(Mecanum drive,
                       MotionController motion,
                       Shooter shooter,
                       ShooterYaw shooterYaw,
                       Spindexer spindexer,
                       Intake intake,
                       Transfer transfer,
                       SlotColorSensors slots,
                       InventoryManager inv,
                       HeadingTarget heading,
                       boolean isRed,
                       Pose shootPose,
                       Pose finalPose,
                       UiLight ui,
                       TelemetryHelper tele) {
        this(drive, motion, shooter, shooterYaw, spindexer, intake, transfer, slots, inv,
                heading, isRed, shootPose, finalPose, ui, tele, Options.defaults());
    }

    public AutoManager(Mecanum drive,
                       MotionController motion,
                       Shooter shooter,
                       ShooterYaw shooterYaw,
                       Spindexer spindexer,
                       Intake intake,
                       Transfer transfer,
                       SlotColorSensors slots,
                       InventoryManager inv,
                       HeadingTarget heading,
                       boolean isRed,
                       Pose shootPose,
                       Pose finalPose,
                       UiLight ui,
                       TelemetryHelper tele,
                       Options options) {
        this.drive = drive;
        this.motion = motion;
        this.shooter = shooter;
        this.shooterYaw = shooterYaw;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.intake = intake;
        this.slots = slots;
        this.inv = inv;
        this.heading = heading;
        this.isRed = isRed;
        this.shootPose = shootPose;
        this.finalPose = finalPose;
        this.ui = ui;
        this.tele = tele;
        this.deposit = new DepositController(shooter, transfer, spindexer, tele);
        this.options = options == null ? Options.defaults() : options;
    }

    public void start(boolean depositRoute) {
        spindexer.setSlot(0);
        shooter.start();
        shooterYaw.start();
        shooter.setAutoRpm(IDLE_RPM);

        shotsRemaining = 3;

        inv.reset();
        SpindexerModel model = inv.getModel();
        model.setBucketContents(0, SpindexerModel.BallColor.BALL);
        model.setBucketContents(1, SpindexerModel.BallColor.BALL);
        model.setBucketContents(2, SpindexerModel.BallColor.BALL);

        if (depositRoute && options.enableDeposit) {
            s = State.PATH_TO_SHOOT;
        } else {
            s = options.enableFinalMove ? State.FINAL_PARK : State.DONE;
        }
        pathIssued = false;
        t.resetTimer();
    }

    public void update() {
        if (slots != null) slots.update();
        if (ui != null) ui.update();
        if (spindexer != null) spindexer.operate();
        if (transfer != null) transfer.operate();

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

            case INTAKING:
                handleIntaking();
                break;

            case STORE_BALL:
                handleStoreBall();
                break;

            case FINAL_PARK:
                handleFinalPark();
                break;

            case DONE:
                handleDone();
                break;
        }

        addTelemetry();
    }

    private void handlePathToShoot() {
        if (!options.enableDeposit) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.NAVIGATING);
        shooterYaw.lockAllianceGoal();
        shooterYaw.operate();

        if (!pathIssued) {
            double headDeg = heading.getTargetHeadingDeg(shootPose);
            motion.followToPose(shootPose, headDeg);
            pathIssued = true;
        }

        // Spin up shooter while moving
        double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));
        shooter.setAutoRpm(rpm);
        shooter.operate();

        if (!motion.isBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_GOAL_S) {
            deposit.reset();
            transitionTo(State.SHOOTING);
        }
    }

    private void handleShooting() {
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
        if (r == DepositController.Result.SHOT) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.SHOT, 300);
            inv.onShot();
            shotsRemaining--;

            if (shotsRemaining > 0) {
                // More balls to shoot
                transitionTo(State.ROTATE_NEXT_BALL);
            } else if (options.enableIntake && inv.setsRemain()) {
                // All shot, go intake more
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                // Done shooting, no intake available
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        } else if (r == DepositController.Result.FAIL) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.FAIL, 500);
            // Even on failure, decrement and continue
            shotsRemaining--;
            if (shotsRemaining > 0) {
                transitionTo(State.ROTATE_NEXT_BALL);
            } else if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        }
    }

    private void handleRotateNextBall() {
        // Find next ball and rotate to it
        int nextBall = inv.decideTargetSlot(slots, spindexer);
        if (nextBall >= 0 && nextBall != spindexer.getCurrentSlot()) {
            spindexer.setSlot(nextBall);
        }

        // Wait for spindexer to settle
        if (spindexer.isSettled()) {
            deposit.reset();
            transitionTo(State.SHOOTING);
        }
    }

    private void handlePathToIntake() {
        if (!options.enableIntake) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);

        if (!pathIssued) {
            Pose target = inv.nextIntakePose(isRed);
            double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
            motion.followToPose(target, headDeg);
            pathIssued = true;
        }

        if (!motion.isBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_INTAKE_S) {
            inv.markOneIntakeSetVisited();
            transitionTo(State.ALIGN_EMPTY_SLOT);
        }
    }

    private void handleAlignEmptySlot() {
        int emptySlot = inv.findNearestEmptySlot(slots, spindexer);
        if (emptySlot >= 0 && emptySlot != spindexer.getCurrentSlot()) {
            spindexer.setSlot(emptySlot);
        }

        if (spindexer.isSettled()) {
            intake.setAutoMode(Intake.AutoMode.FORWARD);
            transitionTo(State.INTAKING);
        }
    }

    private void handleIntaking() {
        if (!options.enableIntake) {
            drive.clearAutoCommand();
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);

        // Creep forward while intaking
        motion.translateFacing(0, INTAKE_FORWARD_SPEED, heading);
        intake.setAutoMode(Intake.AutoMode.FORWARD);
        intake.operate();

        // Check for ball
        boolean gotBall = slots != null && slots.hasBall();
        boolean timeout = t.getElapsedTimeSeconds() >= INTAKE_FORWARD_TIMEOUT_S;

        if (gotBall) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.PICKUP, 250);
            intake.setAutoMode(Intake.AutoMode.OFF);
            drive.clearAutoCommand();
            inv.onBallIntaked();
            transitionTo(State.STORE_BALL);
        } else if (timeout) {
            // Timeout
            intake.setAutoMode(Intake.AutoMode.OFF);
            drive.clearAutoCommand();

            if (inv.hasBalls() && options.enableDeposit) {
                // Have some balls, go shoot them
                shotsRemaining = inv.getBallCount();
                transitionTo(State.PATH_TO_SHOOT);
            } else if (inv.setsRemain()) {
                // Try next intake spot
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                // No more spots, go park
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        }
    }

    private void handleStoreBall() {
        // Raise transfer lever to secure ball
        transfer.raiseLever();

        // Check if we have more empty slots and intake spots
        if (inv.hasEmptySlots() && inv.setsRemain()) {
            // Rotate to next empty and continue intaking
            transitionTo(State.ALIGN_EMPTY_SLOT);
        } else if (inv.hasBalls() && options.enableDeposit) {
            // Full or no more spots - go shoot
            shotsRemaining = inv.getBallCount();
            transitionTo(State.PATH_TO_SHOOT);
        } else {
            // Nothing to do
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
        }
    }

    private void handleFinalPark() {
        if (!options.enableFinalMove) {
            transitionTo(State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.PARK);

        if (!pathIssued) {
            double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
            motion.followToPose(finalPose, headDeg);
            pathIssued = true;
        }

        if (!motion.isBusy() || t.getElapsedTimeSeconds() >= DEFAULT_TIMEOUT_S) {
            transitionTo(State.DONE);
        }
    }

    private void handleDone() {
        if (ui != null) ui.setBase(UiLightConfig.UiState.DONE);
        drive.setAutoDrive(0, 0, 0, true, 0);
        shooter.setAutoRpm(0);
        shooter.operate();
    }

    private void transitionTo(State newState) {
        s = newState;
        pathIssued = false;
        t.resetTimer();
    }

    private void addTelemetry() {
        tele.addLine("=== AUTO ===")
                .addData("State", s::name)
                .addData("t", "%.2f", t.getElapsedTimeSeconds())
                .addData("ShotsLeft", "%d", shotsRemaining)
                .addData("Balls", "%d", inv.getBallCount());
    }
}
