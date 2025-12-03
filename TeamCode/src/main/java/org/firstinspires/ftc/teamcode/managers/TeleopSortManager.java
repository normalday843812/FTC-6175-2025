package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_FEED_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SHIFT_CONFIRM_MS;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TELEOP_SHOOT_SPINUP_MS;
import static org.firstinspires.ftc.teamcode.config.ColorCal.COLOR_MARGIN;
import static org.firstinspires.ftc.teamcode.config.ColorCal.MIN_BALL_CONF;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class TeleopSortManager {

    private enum State {
        DISABLED,
        INTAKING,
        LIFT_AND_LOAD,
        INDEXING,
        SPINDEX_TRANSIT,
        RESET_LEVER,
        FULL_HOLD,
        SHOOTING_STAGING,
        SHOOTING_FLICK,
        SHOOTING_RESET,
        SHOOTING_HOLD,
        FIND_COLOR
    }

    private final GamepadMap map;
    private final Intake intake;
    private final Spindexer spindexer;
    private final Transfer transfer;
    private final SlotColorSensors slots;
    private final InventoryManager inv;
    private final Shooter shooter;
    private final TelemetryHelper tele;

    private State state;
    private boolean enabled;
    private long tMs;
    private int ballConfirmationCount = 0;

    public TeleopSortManager(GamepadMap map, Intake intake, Spindexer spindexer, Transfer transfer,
                             SlotColorSensors slots, InventoryManager inv, Shooter shooter, OpMode opmode) {
        this.map = map;
        this.intake = intake;
        this.spindexer = spindexer;
        this.transfer = transfer;
        this.shooter = shooter;
        this.slots = slots;
        this.inv = inv;
        enabled = false;
        state = State.INTAKING;
        tMs = now();
        tele = new TelemetryHelper(opmode, true);
    }

    public void update() {
        addTelemetry();
        if (!enabled) return;

        boolean isShooting = (state == State.SHOOTING_HOLD);

        if (map.transferButton && !isShooting) {
            state = State.SHOOTING_STAGING;
            tMs = now();
            shooter.resetShotLogic();
            transfer.holdUp();
        } else if (!isShooting) {
            if (state != State.LIFT_AND_LOAD && state != State.INDEXING) {
                if (map.findGreenBall) {
                    state = State.FIND_COLOR;
                    tMs = now();
                }
                if (map.findPurpleBall) {
                    state = State.FIND_COLOR;
                    tMs = now();
                }
            }
        }

        switch (state) {
            case DISABLED:
                intake.setAutoMode(Intake.AutoMode.OFF);
                transfer.runTransfer(Transfer.CrState.OFF);
                break;

            case INTAKING:
                if (!map.shootingModeToggleHeld) {
                    transfer.lowerLever();
                } else {
                    transfer.raiseLever();
                }
                transfer.runTransfer(Transfer.CrState.REVERSE);
                intake.setAutoMode(Intake.AutoMode.REVERSE);

                SlotColorSensors.Observation observation = slots.getObservation(0);
                boolean potentialBall = observation.valid && observation.ballConfidence >= MIN_BALL_CONF &&
                        Math.abs(observation.purpleConfidence - observation.greenConfidence) >= COLOR_MARGIN;

                if (potentialBall) {
                    ballConfirmationCount++;
                } else {
                    ballConfirmationCount = 0;
                }

                if (ballConfirmationCount > 2) {
                    inv.setWantPurple(observation.color == SlotColorSensors.BallColor.PURPLE);

                    ballConfirmationCount = 0;

                    state = State.LIFT_AND_LOAD;
                    tMs = now();
                }
                break;

            case LIFT_AND_LOAD:
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.FORWARD);

                boolean timeExpired = since(tMs) >= (long) (TELEOP_FEED_DWELL_S * 1000);

                if (timeExpired) {
                    state = State.INDEXING;
                    tMs = now();
                }
                break;

            case INDEXING:
                int target = inv.findNearestEmptySlot(slots, spindexer);
                if (target == -1) {
                    state = State.FULL_HOLD;
                    tMs = now();
                } else {
                    if (!transfer.isLeverRaised()) transfer.raiseLever();

                    // If we need to move, go to transit state first
                    if (target != spindexer.getCurrentSlot()) {
                        spindexer.setSlot(target);
                        state = State.SPINDEX_TRANSIT; // NEW STATE
                        tMs = now();
                    } else {
                        // Already there? Go straight to checking confirmation
                        if (since(tMs) >= TELEOP_SHIFT_CONFIRM_MS) {
                            state = State.RESET_LEVER;
                            tMs = now();
                        }
                    }
                }
                break;

            case SPINDEX_TRANSIT:
                // Wait for physical servo to move (e.g., 250ms)
                if (since(tMs) >= 250) {
                    // Now verify/confirm
                    state = State.RESET_LEVER;
                    tMs = now();
                }
                break;

            case RESET_LEVER:
                transfer.lowerLever();
                state = State.INTAKING;
                ballConfirmationCount = 0;
                tMs = now();
                break;

            case FULL_HOLD:
                intake.setAutoMode(Intake.AutoMode.OFF);
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.FORWARD);

                if (inv.findNearestEmptySlot(slots, spindexer) != -1) {
                    state = State.INTAKING;
                    ballConfirmationCount = 0;
                    tMs = now();
                }
                break;

            case SHOOTING_STAGING:
                transfer.raiseLever();
                transfer.runTransfer(Transfer.CrState.FORWARD);
                if (since(tMs) >= TELEOP_SHOOT_SPINUP_MS) {
                    transfer.flick();
                    state = State.SHOOTING_FLICK;
                    tMs = now();
                }
                break;

            case SHOOTING_FLICK:
                if (since(tMs) >= 250) {
                    inv.onShot();
                    state = State.SHOOTING_RESET;
                    tMs = now();
                }
                break;

            case SHOOTING_RESET:
                transfer.lowerLever();
                transfer.runTransfer(Transfer.CrState.REVERSE);

                if (since(tMs) >= 200) {
                    state = State.INTAKING;
                    ballConfirmationCount = 0;
                    tMs = now();
                }
                break;

            case SHOOTING_HOLD:
                // Keep CR servo running forward
                transfer.runTransfer(Transfer.CrState.FORWARD);
                // Note: transfer.holdUp() was called during transition; the Transfer subsystem FSM handles the servo position.

                boolean shotDetected = shooter.shotOccurred();
                boolean buttonReleased = !map.transferButtonHeld; // Use the raw held state

                // Requirement: Keep raised UNTIL shot detected OR button released
                if (shotDetected || buttonReleased) {
                    if (shotDetected) {
                        // Requirement: Only count the shot if RPM drop occurred
                        inv.onShot();
                    }
                    // Requirement: If either condition is true, then drop down.
                    transfer.releaseHold();

                    // Transition back to standard operation.
                    // The Transfer FSM (FLICK_DOWN -> IDLE) handles the physical downward motion.
                    state = State.INTAKING;
                    ballConfirmationCount = 0;
                    tMs = now();
                }
                break;

            case FIND_COLOR:
                SlotColorSensors.BallColor want =
                        map.findGreenBall ? SlotColorSensors.BallColor.GREEN :
                                (map.findPurpleBall ? SlotColorSensors.BallColor.PURPLE
                                        : SlotColorSensors.BallColor.NONE);

                if (want == SlotColorSensors.BallColor.NONE) {
                    state = State.INTAKING;
                    ballConfirmationCount = 0;
                    tMs = now();
                    break;
                }

                int idx = slots.findBallSlot(want);
                if (idx >= 0) {
                    transfer.lowerLever();
                    spindexer.setSlot(idx);
                }
                break;
        }
    }

    public void setEnabled(boolean enabled) {
        // If the status isn't changing, do nothing (Fixes the reset bug).
        if (this.enabled == enabled) {
            return;
        }
        this.enabled = enabled;

        if (!enabled) {
            state = State.DISABLED;
            // Switch subsystems back to manual control (sets mode = MANUAL)
            intake.startTeleop();
            spindexer.startTeleop();
            transfer.startTeleop(); // Fixes the double flick
        } else {
            // Switch subsystems to auto control (sets mode = AUTO)
            intake.startAuto();
            spindexer.startAuto();
            transfer.startAuto(); // Fixes the double flick

            // Initialize the manager's state machine
            state = State.INTAKING;
            ballConfirmationCount = 0;
            tMs = now();
        }
    }

    public boolean getEnabled() { return enabled; }
    private static long now() { return System.nanoTime() / 1_000_000L; }
    private static long since(long startMs) { return now() - startMs; }

    private void addTelemetry() {
        tele.addLine("=== SYSTEM STATUS ===");
        tele.addData("Enabled", "%b", enabled);
        tele.addData("State", "%s (%.2fs)", state, since(tMs) / 1000.0);
        tele.addData("Loop Time", "%.1f ms", (System.nanoTime() - (tMs * 1_000_000)) / 1_000_000.0);

        SlotColorSensors.Observation obs = slots.getObservation(0);

        tele.addLine("\n=== SENSOR READINGS ===");
        tele.addData("Valid Reading", "%b", obs.valid);

        boolean confThresholdMet = obs.ballConfidence >= MIN_BALL_CONF;
        double abs = Math.abs(obs.purpleConfidence - obs.greenConfidence);
        boolean colorMarginMet = abs >= COLOR_MARGIN;
        boolean potentialBall = obs.valid && confThresholdMet && colorMarginMet;

        tele.addLine("\n=== SORTING LOGIC ===");
        tele.addData("1. Conf > Min?", "%b (%.2f > %.2f)", confThresholdMet, obs.ballConfidence, MIN_BALL_CONF);
        tele.addData("2. Margin Ok?", "%b (Diff: %.3f)", colorMarginMet, abs);
        tele.addData("3. Potential Ball?", "%b", potentialBall);
        tele.addData("Confirmation Count", "%d / 3", ballConfirmationCount);

        tele.addLine("\n=== INVENTORY ===");
        tele.addData("Want Purple?", "%b", inv.findNearestEmptySlot(slots, spindexer));
    }
}