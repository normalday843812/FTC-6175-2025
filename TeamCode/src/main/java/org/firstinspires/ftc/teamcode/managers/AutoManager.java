package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_CREEP_DISTANCE;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_FORWARD_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_FEED_DWELL_S;
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
import com.pedropathing.geometry.Pose;
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
    private State s;
    private int shotsRemaining = 3;
    private boolean pathIssued = false;

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

        boolean shooterHandledByState = (s == State.PATH_TO_SHOOT || s == State.SHOOTING);
        if (!shooterHandledByState) {
            shooter.setAutoRpm(IDLE_RPM);
            shooter.operate();
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
        transfer.raiseLever();
        if (!options.enableDeposit) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.NAVIGATING);
        shooterYaw.lockAllianceGoal();
        shooterYaw.operate();

        if (!pathIssued) {
            double headDeg = getHeadingToGoal(shootPose);
            followToPose(shootPose, headDeg);
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
            // Sync model with current spindexer position before marking shot
            inv.getModel().setBucketAtFront(spindexer.getCurrentSlot());
            inv.onShot();
            shotsRemaining--;

            if (shotsRemaining > 0) {
                transitionTo(State.ROTATE_NEXT_BALL);
            } else if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        } else if (r == DepositController.Result.NO_BALL) {
            // Sensors say no ball at front - mark front bucket empty and try to find another
            if (ui != null) ui.notify(UiLightConfig.UiEvent.FAIL, 300);
            inv.getModel().setBucketContents(spindexer.getCurrentSlot(), SpindexerModel.BallColor.EMPTY);
            shotsRemaining--;

            if (shotsRemaining > 0 && inv.hasBalls()) {
                // Model still thinks there are balls - try rotating to find one
                transitionTo(State.ROTATE_NEXT_BALL);
            } else if (options.enableIntake && inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        } else if (r == DepositController.Result.FAIL) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.FAIL, 500);
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
        transfer.raiseLever();
        int nextBall = inv.decideTargetSlot(spindexer);
        if (nextBall >= 0 && nextBall != spindexer.getCurrentSlot()) {
            spindexer.setSlot(nextBall);
        }

        if (spindexer.isSettled()) {
            if (slots != null && !slots.hasBall()) {
                inv.getModel().setBucketContents(spindexer.getCurrentSlot(), SpindexerModel.BallColor.EMPTY);

                if (!inv.hasBalls()) {
                    if (options.enableIntake && inv.setsRemain()) {
                        transitionTo(State.PATH_TO_INTAKE);
                    } else {
                        transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
                    }
                }
                return;
            }

            deposit.reset();
            transitionTo(State.SHOOTING);
        }
    }

    private void handlePathToIntake() {
        transfer.lowerLever();
        if (!options.enableIntake) {
            transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            return;
        }

        if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);

        if (!pathIssued) {
            Pose target = inv.nextIntakePose(isRed);
            double intakeHeadingDeg = Math.toDegrees(target.getHeading());
            followToPose(target, intakeHeadingDeg);
            pathIssued = true;
        }

        if (!drive.isPathBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_INTAKE_S) {
            inv.markOneIntakeSetVisited();
            transitionTo(State.ALIGN_EMPTY_SLOT);
        }
    }

    private void handleAlignEmptySlot() {
        int emptySlot = inv.findNearestEmptySlot(spindexer);

        if (emptySlot < 0 && slots != null && !slots.hasBall()) {
            inv.getModel().setBucketContents(spindexer.getCurrentSlot(), SpindexerModel.BallColor.EMPTY);
            emptySlot = spindexer.getCurrentSlot();
        }

        if (emptySlot >= 0 && emptySlot != spindexer.getCurrentSlot()) {
            spindexer.setSlot(emptySlot);
        }

        if (spindexer.isSettled()) {
            intake.setAutoMode(Intake.AutoMode.FORWARD);
            transitionTo(State.INTAKING);
        }
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
            followToPose(creepTarget, Math.toDegrees(current.getHeading()));
            pathIssued = true;
        }

        intake.setAutoMode(Intake.AutoMode.FORWARD);
        intake.operate();

        boolean gotBall = slots != null && slots.hasBall();
        boolean timeout = t.getElapsedTimeSeconds() >= INTAKE_FORWARD_TIMEOUT_S;
        boolean pathDone = !drive.isPathBusy();

        if (gotBall) {
            if (ui != null) ui.notify(UiLightConfig.UiEvent.PICKUP, 250);
            intake.setAutoMode(Intake.AutoMode.OFF);
            inv.getModel().setBucketAtFront(spindexer.getCurrentSlot());
            inv.onBallIntaked();
            transitionTo(State.STORE_BALL);
        } else if (timeout || pathDone) {
            intake.setAutoMode(Intake.AutoMode.OFF);

            if (inv.hasBalls() && options.enableDeposit) {
                shotsRemaining = inv.getBallCount();
                transitionTo(State.PATH_TO_SHOOT);
            } else if (inv.setsRemain()) {
                transitionTo(State.PATH_TO_INTAKE);
            } else {
                transitionTo(options.enableFinalMove ? State.FINAL_PARK : State.DONE);
            }
        }
    }

    private void handleStoreBall() {
        transfer.raiseLever();
        transfer.runTransfer(Transfer.CrState.FORWARD);

        // Wait for ball to be pushed into bucket before deciding next action
        if (t.getElapsedTimeSeconds() < TELEOP_FEED_DWELL_S) {
            return;
        }

        // Continue intaking at current position if we have empty slots
        if (inv.hasEmptySlots()) {
            transitionTo(State.ALIGN_EMPTY_SLOT);
        } else if (inv.hasBalls() && options.enableDeposit) {
            // Full - go shoot
            shotsRemaining = inv.getBallCount();
            transitionTo(State.PATH_TO_SHOOT);
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

        if (ui != null) ui.setBase(UiLightConfig.UiState.PARK);

        if (!pathIssued) {
            double headDeg = Math.toDegrees(finalPose.getHeading());
            followToPose(finalPose, headDeg);
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
        shooter.setAutoRpm(0);
        shooter.operate();
    }

    private void transitionTo(State newState) {
        s = newState;
        pathIssued = false;
        t.resetTimer();
    }

    // --- Path Helpers (using Pedro directly) ---

    private void followToPose(Pose target, double headDeg) {
        Pose current = follower.getPose();
        Pose control = midpointControl(current, target);

        PathChain chain = follower.pathBuilder()
                .addPath(new BezierCurve(follower::getPose, control, target))
                .setLinearHeadingInterpolation(
                        current.getHeading(),
                        Math.toRadians(headDeg),
                        0.8
                )
                .build();

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
                .addData("ShotsLeft", "%d", shotsRemaining)
                .addData("Balls", "%d", inv.getBallCount());
    }
}
