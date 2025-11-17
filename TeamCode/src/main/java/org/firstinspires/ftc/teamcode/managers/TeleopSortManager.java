package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DELTA_DOWN;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DELTA_UP;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_MAX;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_FEED_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SHIFT_CONFIRM_MS;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SHOOT_SPINUP_MS;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_WAIT_BALL_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.ColorCal.COLOR_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.MIN_BALL_CONF;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class TeleopSortManager {

    private enum State {
        DISABLED,
        IDLE,
        INTAKE_ARM,
        WAIT_SLOT0_BALL,
        FEED_TO_SPX,
        SHIFT_PREP,
        SHIFT_TO_SLOTS,
        FULL_BEHAVIOR,
        SHOOT_STAGING,
        SHOOT_EXEC,
        VERIFY_SHOT,
        FIND_COLOR,
        RECOVER
    }

    private final GamepadMap map;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final SlotColorSensors slots;
    private final InventoryManager inv;

    private State state;
    private boolean enabled;

    private long tMs;
    private int recoverJiggles = 0;

    public TeleopSortManager(GamepadMap map, Intake intake, Spindexer spindexer, Transfer transfer,
                             SlotColorSensors slots, InventoryManager inv) {
        this.map = map;
        this.intake = intake;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.slots = slots;
        this.inv = inv;
        enabled = true;
        state = State.IDLE;
        tMs = now();
    }

    public void update() {
        if (!enabled) return;

        if (map.findGreenBall && (state == State.IDLE || state == State.INTAKE_ARM)) {
            state = State.FIND_COLOR;
            tMs = now();
        } else if (map.findPurpleBall && (state == State.IDLE || state == State.INTAKE_ARM)) {
            state = State.FIND_COLOR;
            tMs = now();
        }

        switch (state) {
            case DISABLED:
                break;

            case IDLE:
                transfer.lowerLever();
                transfer.runTransfer(Transfer.CrState.OFF);
                state = State.INTAKE_ARM;
                tMs = now();
                break;

            case INTAKE_ARM:
                transfer.lowerLever();
                transfer.runTransfer(Transfer.CrState.REVERSE);
                intake.setAutoMode(Intake.AutoMode.FORWARD);
                state = State.WAIT_SLOT0_BALL;
                tMs = now();
                break;

            case WAIT_SLOT0_BALL: {
                SlotColorSensors.Observation observation = slots.getObservation(0);
                boolean ok = observation.valid && observation.ballConfidence >= MIN_BALL_CONF &&
                        Math.abs(observation.purpleConfidence - observation.greenConfidence) >= COLOR_MARGIN;
                if (ok) {
                    inv.setWantPurple(observation.color == SlotColorSensors.BallColor.PURPLE);
                    state = State.FEED_TO_SPX;
                    tMs = now();
                } else if (since(tMs) >= (long) (TELEOP_WAIT_BALL_TIMEOUT_S * 1000)) {
                    state = State.RECOVER;
                    tMs = now();
                    recoverJiggles = 0;
                }
                break;
            }

            case FEED_TO_SPX:
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.FORWARD);
                if (since(tMs) >= (long) (TELEOP_FEED_DWELL_S * 1000)) {
                    transfer.runTransfer(Transfer.CrState.OFF);
                    state = State.SHIFT_PREP;
                    tMs = now();
                }
                break;

            case SHIFT_PREP:
                transfer.lowerLever();
                state = State.SHIFT_TO_SLOTS;
                tMs = now();
                break;

            case SHIFT_TO_SLOTS: {
                int target = inv.findNearestEmptySlot(slots, spindexer);
                if (target >= 0) {
                    if (!transfer.isLeverRaised() && transfer.isIdle()) {
                        spindexer.setSlot(target);
                        if (since(tMs) >= TELEOP_SHIFT_CONFIRM_MS) {
                            state = State.INTAKE_ARM;
                            tMs = now();
                        }
                    }
                } else {
                    state = State.FULL_BEHAVIOR;
                    tMs = now();
                }
                break;
            }

            case FULL_BEHAVIOR:
                intake.setAutoMode(Intake.AutoMode.OFF);
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.FORWARD);
                if (map.transferButton) {
                    state = State.SHOOT_STAGING;
                    tMs = now();
                }
                break;

            case SHOOT_STAGING:
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.FORWARD);
                if (since(tMs) >= TELEOP_SHOOT_SPINUP_MS) {
                    transfer.flick();
                    state = State.SHOOT_EXEC;
                    tMs = now();
                }
                break;

            case SHOOT_EXEC:
                if (transfer.isIdle()) {
                    state = State.VERIFY_SHOT;
                    tMs = now();
                }
                break;

            case VERIFY_SHOT:
                // Shooter shot detection is event-based; also time out
                if (since(tMs) >= 500) {
                    inv.onShot();
                    transfer.lowerLever();
                    transfer.runTransfer(Transfer.CrState.REVERSE);
                    state = State.INTAKE_ARM;
                    tMs = now();
                }
                break;

            case FIND_COLOR:
                SlotColorSensors.BallColor want =
                        map.findGreenBall ? SlotColorSensors.BallColor.GREEN :
                                (map.findPurpleBall ? SlotColorSensors.BallColor.PURPLE
                                        : SlotColorSensors.BallColor.NONE);
                if (want == SlotColorSensors.BallColor.NONE) {
                    state = State.IDLE;
                    tMs = now();
                    break;
                }
                int idx = slots.findBallSlot(want);
                if (idx >= 0) {
                    transfer.lowerLever();
                    if (transfer.isIdle()) {
                        spindexer.setSlot(idx);
                        state = State.IDLE;
                        tMs = now();
                    }
                } else {
                    state = State.IDLE;
                    tMs = now();
                }
                break;

            case RECOVER:
                if (recoverJiggles < JIGGLE_MAX && !transfer.isLeverRaised() && transfer.isIdle()) {
                    spindexer.startJiggle(JIGGLE_DELTA_UP, JIGGLE_DELTA_DOWN, JIGGLE_DWELL_S);
                    if (spindexer.updateJiggle()) {
                        recoverJiggles++;
                        tMs = now();
                    }
                } else {
                    intake.setAutoMode(Intake.AutoMode.OFF);
                    transfer.runTransfer(Transfer.CrState.OFF);
                    state = State.IDLE;
                    tMs = now();
                }
                break;
        }
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) state = State.DISABLED;
        else {
            state = State.IDLE;
            tMs = now();
        }
    }

    public boolean getEnabled() {
        return enabled;
    }

    private static long now() {
        return System.nanoTime() / 1_000_000L;
    }

    private static long since(long startMs) {
        return now() - startMs;
    }
}
