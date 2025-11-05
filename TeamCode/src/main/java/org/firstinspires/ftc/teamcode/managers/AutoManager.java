package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_GPP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PGP;
import static org.firstinspires.ftc.teamcode.config.AutoConfig.APRIL_TAG_PPG;
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
import org.firstinspires.ftc.teamcode.shooting.PolynomialRpmModel;
import org.firstinspires.ftc.teamcode.shooting.RpmModel;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterYaw;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSlotsColor;
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

    private final TelemetryHelper tele;
    private final Mecanum drive;
    private final MotionController motion;
    private final Shooter shooter;
    private final ShooterYaw shooterYaw;
    private final Spindexer spindexer;
    private final Intake intake;
    private final SpindexSlotsColor slots;
    private final InventoryManager inv;
    private final DistanceSubsystem distance;

    private final HeadingTarget heading;
    private final boolean isRed;
    private final Pose shootPose, finalPose;

    private final DepositController deposit;
    private final RpmModel rpmModel = new PolynomialRpmModel();
    private final Timer t = new Timer();
    private State s;

    private final UiLight ui;

    public AutoManager(Mecanum drive,
                       MotionController motion,
                       Shooter shooter,
                       ShooterYaw shooterYaw,
                       Spindexer spindexer,
                       Intake intake,
                       Transfer transfer,
                       SpindexSlotsColor slots,
                       InventoryManager inv,
                       DistanceSubsystem distance,
                       HeadingTarget heading,
                       boolean isRed,
                       Pose shootPose,
                       Pose finalPose,
                       UiLight ui,
                       TelemetryHelper tele) {
        this.drive = drive;
        this.motion = motion;
        this.shooter = shooter;
        this.shooterYaw = shooterYaw;
        this.spindexer = spindexer;
        this.intake = intake;
        this.slots = slots;
        this.inv = inv;
        this.distance = distance;
        this.heading = heading;
        this.isRed = isRed;
        this.shootPose = shootPose;
        this.finalPose = finalPose;
        this.ui = ui;
        this.tele = tele;
        this.deposit = new DepositController(shooter, transfer, spindexer, tele);
    }

    public void start(boolean depositRoute) {
        spindexer.setSlot(0);
        shooter.startAuto();
        intake.startAuto();
        shooterYaw.startAuto();
        shooter.setAutoRpm(IDLE_RPM);
        s = depositRoute ? State.SEEK_PATTERN : State.FINAL_MOVE;
        t.resetTimer();
    }

    public void update() {
        slots.update();
        distance.update();
        ui.update();

        switch (s) {
            case SEEK_PATTERN:
                ui.setBase(UiLightConfig.UiState.SEEKING);
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
                ui.setBase(UiLightConfig.UiState.DECIDING);
                int slot = inv.decideTargetSlot(slots, spindexer);
                if (slot >= 0 && slot != spindexer.getCurrentSlot()) spindexer.setSlot(slot);
                if (slot >= 0) {
                    s = State.PATH_TO_GOAL;
                    t.resetTimer();
                } else {
                    s = inv.setsRemain() ? State.INTAKE_GOTO_SET : State.FINAL_MOVE;
                    t.resetTimer();
                }
                break;
            }


            case PATH_TO_GOAL: {
                ui.setBase(UiLightConfig.UiState.NAVIGATING);
                shooterYaw.lockAllianceGoal();
                shooterYaw.operate();

                if (t.getElapsedTimeSeconds() == 0) {
                    double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
                    motion.followToPose(shootPose, headDeg);
                }

                Pose robot = drive.getFollower().getPose();
                double rpm = rpmModel.isValid()
                        ? rpmModel.computeTargetRpm(robot, shootPose)
                        : IDLE_RPM;
                shooter.setAutoRpm(Math.min(MAX_RPM, Math.max(IDLE_RPM, rpm)));
                shooter.operate();

                if (!motion.isBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_GOAL_S) {
                    deposit.reset();
                    s = State.DEPOSIT;
                    t.resetTimer();
                }
                break;
            }


            case DEPOSIT: {
                // spinup vs ready
                ui.setBase(shooter.isAtTarget(TARGET_RPM_BAND)
                        ? UiLightConfig.UiState.READY
                        : UiLightConfig.UiState.SPINUP);

                Pose robot = drive.getFollower().getPose();
                double rpm = rpmModel.isValid()
                        ? rpmModel.computeTargetRpm(robot, shootPose)
                        : IDLE_RPM;

                DepositController.Result r = deposit.update(rpm);
                if (r == DepositController.Result.SHOT) {
                    ui.notify(UiLightConfig.UiEvent.SHOT, 300);
                    inv.onShot();
                    if (slots.hasAnyBall(0) || slots.hasAnyBall(1) || slots.hasAnyBall(2)) {
                        s = State.SPINDEX_DECIDE;
                        t.resetTimer();
                    } else if (inv.setsRemain()) {
                        s = State.INTAKE_GOTO_SET;
                        t.resetTimer();
                    } else {
                        s = State.FINAL_MOVE;
                        t.resetTimer();
                    }
                } else if (r == DepositController.Result.FAIL) {
                    ui.notify(UiLightConfig.UiEvent.FAIL, 500);
                    s = State.FINAL_MOVE;
                    t.resetTimer();
                }
                break;
            }

            case INTAKE_GOTO_SET: {
                ui.setBase(UiLightConfig.UiState.INTAKE);
                if (t.getElapsedTimeSeconds() == 0) {
                    Pose target = inv.nextIntakePose(isRed);
                    double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
                    motion.followToPose(target, headDeg);
                }

                if (!motion.isBusy() || t.getElapsedTimeSeconds() >= PATH_TIMEOUT_TO_INTAKE_S) {
                    inv.markOneIntakeSetVisited();
                    intake.setAutoMode(Intake.AutoMode.FORWARD);
                    s = State.INTAKE_FORWARD;
                    t.resetTimer();
                }
                break;
            }

            case INTAKE_FORWARD: {
                ui.setBase(UiLightConfig.UiState.INTAKE);
                motion.translateFacing(0, INTAKE_FORWARD_SPEED, heading);
                intake.setAutoMode(Intake.AutoMode.FORWARD);
                intake.operate();

                boolean gotBall = slots.hasAnyBall(0);
                boolean wallClose = distance.isWallClose();
                boolean timeout = t.getElapsedTimeSeconds() >= INTAKE_FORWARD_TIMEOUT_S;

                if (gotBall) {
                    ui.notify(UiLightConfig.UiEvent.PICKUP, 250);
                    s = State.SPINDEX_DECIDE;
                    t.resetTimer();
                } else if (timeout && !(slots.hasAnyBall(0) || slots.hasAnyBall(1) || slots.hasAnyBall(2))) {
                    s = State.FINAL_MOVE;
                    t.resetTimer();
                } else if (timeout || wallClose) {
                    s = State.PATH_TO_GOAL;
                    t.resetTimer();
                }
                break;
            }

            case FINAL_MOVE: {
                ui.setBase(UiLightConfig.UiState.PARK);
                if (t.getElapsedTimeSeconds() == 0) {
                    double headDeg = heading.getTargetHeadingDeg(drive.getFollower().getPose());
                    motion.followToPose(finalPose, headDeg);
                }

                if (!motion.isBusy() || t.getElapsedTimeSeconds() >= DEFAULT_TIMEOUT_S) {
                    s = State.DONE;
                }
                break;
            }


            case DONE:
                ui.setBase(UiLightConfig.UiState.DONE);
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
