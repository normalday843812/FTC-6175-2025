package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_GPP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PGP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PPG;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AUTO_TARGET_RPM;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DEFAULT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_FORWARD_SPEED;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INTAKE_FORWARD_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATH_TIMEOUT_TO_GOAL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATH_TIMEOUT_TO_INTAKE_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATTERN_SEEK_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.PATTERN_STABLE_HOLD_MS;
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
        SEEK_PATTERN,
        SPINDEX_DECIDE,
        PATH_TO_GOAL,
        DEPOSIT,
        INTAKE_GOTO_SET,
        INTAKE_FORWARD,
        FINAL_MOVE,
        DONE
    }

    public static final class Options {
        public final boolean enablePatternSeek;
        public final boolean enableIntake;
        public final boolean enableDeposit;
        public final boolean enableFinalMove;
        public final boolean useColorSensors;

        public Options(boolean enablePatternSeek,
                       boolean enableIntake,
                       boolean enableDeposit,
                       boolean enableFinalMove,
                       boolean useColorSensors) {
            this.enablePatternSeek = enablePatternSeek;
            this.enableIntake = enableIntake;
            this.enableDeposit = enableDeposit;
            this.enableFinalMove = enableFinalMove;
            this.useColorSensors = useColorSensors;
        }

        public static Options defaults() {
            return new Options(true, true, true, true, true);
        }
    }

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
    private final Timer t = new Timer();
    private State s;

    private final UiLight ui;
    private final Options options;

    private boolean pathToGoalIssued = false;
    private boolean intakePathIssued = false;
    private boolean finalPathIssued = false;

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
        if (this.slots != null) {
            this.slots.setEnabled(this.options.useColorSensors);
        }
    }

    public void start(boolean depositRoute) {
        spindexer.setSlot(0);
        shooter.startAuto();
        intake.startAuto();
        shooterYaw.startAuto();
        transfer.startAuto();
        shooter.setAutoRpm(IDLE_RPM);
        boolean wantDeposit = depositRoute && options.enableDeposit;
        if (wantDeposit) {
            s = options.enablePatternSeek ? State.SEEK_PATTERN : State.SPINDEX_DECIDE;
        } else {
            s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
        }
        pathToGoalIssued = false;
        intakePathIssued = false;
        finalPathIssued = false;
        t.resetTimer();
    }

    public void update() {
        if (slots != null) slots.update();
        if (ui != null) ui.update();
        if (spindexer != null) spindexer.operate();
        if (transfer != null) transfer.operate();

        switch (s) {
            case SEEK_PATTERN:
                if (!options.enablePatternSeek) {
                    s = State.SPINDEX_DECIDE;
                    t.resetTimer();
                    break;
                }
                if (ui != null) ui.setBase(UiLightConfig.UiState.SEEKING);
                shooterYaw.seekPattern(new int[]{
                        APRIL_TAG_GPP,
                        APRIL_TAG_PGP,
                        APRIL_TAG_PPG
                }, PATTERN_STABLE_HOLD_MS);
                shooterYaw.operate();
                if (shooterYaw.isPatternChosen()) {
                    inv.setPatternFromTagId(shooterYaw.getChosenPatternId());
                    s = State.SPINDEX_DECIDE;
                    t.resetTimer();
                } else if (t.getElapsedTimeSeconds() >= PATTERN_SEEK_TIMEOUT_S) {
                    s = State.SPINDEX_DECIDE;
                    t.resetTimer();
                }
                break;

            case SPINDEX_DECIDE: {
                if (ui != null) ui.setBase(UiLightConfig.UiState.DECIDING);
                int slot = 0;
                if (spindexer != null) {
                    slot = inv.decideTargetSlot(slots, spindexer);
                }
                if (spindexer != null && slot >= 0 && slot != spindexer.getCurrentSlot())
                    spindexer.setSlot(slot);
                if (slot >= 0 && options.enableDeposit) {
                    s = State.PATH_TO_GOAL;
                    pathToGoalIssued = false;
                } else if (options.enableIntake && inv.setsRemain()) {
                    s = State.INTAKE_GOTO_SET;
                    intakePathIssued = false;
                } else {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    finalPathIssued = false;
                }
                t.resetTimer();
                break;
            }

            case PATH_TO_GOAL: {
                if (!options.enableDeposit) {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    t.resetTimer();
                    finalPathIssued = false;
                    break;
                }
                if (ui != null) ui.setBase(UiLightConfig.UiState.NAVIGATING);
                shooterYaw.lockAllianceGoal();
                shooterYaw.operate();

                if (!pathToGoalIssued) {
                    double headDeg = heading.getTargetHeadingDeg(shootPose);
                    motion.followToPose(shootPose, headDeg);
                    pathToGoalIssued = true;
                }

                double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));
                shooter.setAutoRpm(rpm);
                shooter.operate();

                if (!motion.isBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_GOAL_S) {
                    deposit.reset();
                    s = State.DEPOSIT;
                    t.resetTimer();
                }
                break;
            }

            case DEPOSIT: {
                if (!options.enableDeposit) {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    t.resetTimer();
                    break;
                }

                if (ui != null) {
                    ui.setBase(shooter.isAtTarget(TARGET_RPM_BAND)
                            ? UiLightConfig.UiState.READY
                            : UiLightConfig.UiState.SPINUP);
                }

                double rpm = Math.min(MAX_RPM, Math.max(IDLE_RPM, AUTO_TARGET_RPM));

                DepositController.Result r = deposit.update(rpm);
                if (r == DepositController.Result.SHOT) {
                    if (ui != null) ui.notify(UiLightConfig.UiEvent.SHOT, 300);
                    inv.onShot();
                    boolean hasBall = slots != null && (slots.hasAnyBall(0) || slots.hasAnyBall(1) || slots.hasAnyBall(2));
                    if (hasBall) {
                        s = State.SPINDEX_DECIDE;
                        pathToGoalIssued = false;
                        intakePathIssued = false;
                        finalPathIssued = false;
                    } else if (options.enableIntake && inv.setsRemain()) {
                        s = State.INTAKE_GOTO_SET;
                        intakePathIssued = false;
                    } else {
                        s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                        finalPathIssued = false;
                    }
                    t.resetTimer();
                } else if (r == DepositController.Result.FAIL) {
                    if (ui != null) ui.notify(UiLightConfig.UiEvent.FAIL, 500);
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    finalPathIssued = false;
                    t.resetTimer();
                }
                break;
            }

            case INTAKE_GOTO_SET: {
                if (!options.enableIntake) {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    t.resetTimer();
                    finalPathIssued = false;
                    break;
                }
                if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);
                if (!intakePathIssued) {
                    Pose target = inv.nextIntakePose(isRed);
                    double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
                    motion.followToPose(target, headDeg);
                    intakePathIssued = true;
                }

                if (!motion.isBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_INTAKE_S) {
                    inv.markOneIntakeSetVisited();
                    intake.setAutoMode(Intake.AutoMode.FORWARD);
                    s = State.INTAKE_FORWARD;
                    pathToGoalIssued = false;
                    t.resetTimer();
                }
                break;
            }

            case INTAKE_FORWARD: {
                if (!options.enableIntake) {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    t.resetTimer();
                    break;
                }
                if (ui != null) ui.setBase(UiLightConfig.UiState.INTAKE);
                motion.translateFacing(0, INTAKE_FORWARD_SPEED, heading);
                intake.setAutoMode(Intake.AutoMode.FORWARD);
                intake.operate();

                boolean gotBall = slots != null && slots.hasAnyBall(0);
                boolean timeout = t.getElapsedTimeSeconds() >= INTAKE_FORWARD_TIMEOUT_S;

                if (gotBall) {
                    if (ui != null) ui.notify(UiLightConfig.UiEvent.PICKUP, 250);
                    s = State.SPINDEX_DECIDE;
                    pathToGoalIssued = false;
                    intakePathIssued = false;
                    finalPathIssued = false;
                    t.resetTimer();
                } else if (timeout && !(slots != null && (slots.hasAnyBall(0) || slots.hasAnyBall(1) || slots.hasAnyBall(2)))) {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    finalPathIssued = false;
                    t.resetTimer();
                } else if (timeout && options.enableDeposit) {
                    s = State.PATH_TO_GOAL;
                    pathToGoalIssued = false;
                    t.resetTimer();
                } else if (timeout) {
                    s = options.enableFinalMove ? State.FINAL_MOVE : State.DONE;
                    finalPathIssued = false;
                    t.resetTimer();
                }
                break;
            }

            case FINAL_MOVE: {
                if (!options.enableFinalMove) {
                    s = State.DONE;
                    break;
                }
                if (ui != null) ui.setBase(UiLightConfig.UiState.PARK);
                if (!finalPathIssued) {
                    double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
                    motion.followToPose(finalPose, headDeg);
                    finalPathIssued = true;
                }

                if (!motion.isBusy() || t.getElapsedTimeSeconds() >= DEFAULT_TIMEOUT_S) {
                    s = State.DONE;
                }
                break;
            }

            case DONE:
                if (ui != null) ui.setBase(UiLightConfig.UiState.DONE);
                drive.setAutoDrive(0, 0, 0, true, 0);
                shooter.setAutoRpm(0);
                shooter.operate();
                break;
        }

        tele.addLine("=== AUTO ===")
                .addData("State", s::name)
                .addData("t", "%.2f", t.getElapsedTimeSeconds());
    }
}