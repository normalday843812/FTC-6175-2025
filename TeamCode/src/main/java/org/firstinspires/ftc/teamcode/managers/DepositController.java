package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AT_RPM_WAIT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.DEPOSIT_EMPTY_CONFIRM_CYCLES;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INDEX_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DELTA_DOWN;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DELTA_UP;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_MAX;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.REFIRE_MAX;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.VERIFY_WINDOW_S;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SlotColorSensors;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class DepositController {
    public enum Result {BUSY, SHOT, FAIL, NO_BALL}

    private enum S {SPINUP, WAIT_INDEX, WAIT_FLICK, VERIFY, JIGGLE, DONE, FAIL, NO_BALL}

    private final Shooter shooter;
    private final Transfer transfer;
    private final Spindexer spx;
    private final SlotColorSensors slots;
    private final TelemetryHelper tele;

    private final Timer tState = new Timer();
    private final Timer tVerify = new Timer();

    private S s = S.SPINUP;
    private int refires = 0, jiggles = 0;
    private boolean ballVerified = false;
    private int emptyConfirmCount = 0;
    private boolean sawRpmShot = false;

    public DepositController(Shooter shooter, Transfer transfer, Spindexer spx, SlotColorSensors slots, TelemetryHelper tele) {
        this.shooter = shooter;
        this.transfer = transfer;
        this.spx = spx;
        this.slots = slots;
        this.tele = tele;
        tState.resetTimer();
    }

    public void reset() {
        s = S.SPINUP;
        refires = 0;
        jiggles = 0;
        ballVerified = false;
        emptyConfirmCount = 0;
        sawRpmShot = false;
        tState.resetTimer();
        spx.stopJiggle();
    }

    public Result update(double targetRpm) {
        shooter.setAutoRpm(targetRpm);
        shooter.operate();

        boolean frontHasBall = slots != null && slots.hasFrontBall();

        switch (s) {
            case SPINUP:
                transfer.raiseLever();

                // Verify ball is at front using sensors before proceeding
                if (!ballVerified) {
                    if (frontHasBall) {
                        ballVerified = true;
                    } else if (tState.getElapsedTimeSeconds() >= AT_RPM_WAIT_TIMEOUT_S) {
                        // Timed out waiting for ball - no ball at front
                        s = S.NO_BALL;
                        break;
                    }
                }

                if (ballVerified && (shooter.isAtTarget(TARGET_RPM_BAND) || tState.getElapsedTimeSeconds() >= AT_RPM_WAIT_TIMEOUT_S)) {
                    tState.resetTimer();
                    s = S.WAIT_INDEX;
                }
                break;

            case WAIT_INDEX:
                if (tState.getElapsedTimeSeconds() >= INDEX_DWELL_S) {
                    transfer.flick();
                    s = S.WAIT_FLICK;
                }
                break;

            case WAIT_FLICK:
                if (transfer.isIdle()) {
                    tVerify.resetTimer();
                    emptyConfirmCount = 0;
                    sawRpmShot = false;
                    s = S.VERIFY;
                }
                break;

            case VERIFY:
                if (shooter.shotOccurred()) {
                    sawRpmShot = true;
                    shooter.acknowledgeShotOccurred();
                }

                // Primary truth: slot-0 sensor should clear when the ball leaves.
                if (ballVerified) {
                    if (!frontHasBall) {
                        emptyConfirmCount++;
                        if (emptyConfirmCount >= DEPOSIT_EMPTY_CONFIRM_CYCLES) {
                            if (!sawRpmShot) {
                                // Sensor says ball left but RPM-drop logic didn't trigger (can happen on last ball).
                                // Reset shot logic so next ball can be detected cleanly.
                                shooter.resetShotLogic();
                            }
                            s = S.DONE;
                            break;
                        }
                    } else {
                        emptyConfirmCount = 0;
                    }
                }

                if (tVerify.getElapsedTimeSeconds() >= VERIFY_WINDOW_S) {
                    if (refires < REFIRE_MAX) {
                        refires++;
                        transfer.flick();
                        s = S.WAIT_FLICK;
                    } else if (jiggles < JIGGLE_MAX) {
                        jiggles++;
                        spx.startJiggle(JIGGLE_DELTA_UP, JIGGLE_DELTA_DOWN, JIGGLE_DWELL_S);
                        s = S.JIGGLE;
                    } else {
                        // If the sensor cleared but we didn't get enough consecutive reads (timing), accept as shot.
                        if (ballVerified && !frontHasBall) {
                            shooter.resetShotLogic();
                            s = S.DONE;
                        } else {
                        s = S.FAIL;
                        }
                    }
                }
                break;

            case JIGGLE:
                if (spx.updateJiggle()) {
                    transfer.flick();
                    s = S.WAIT_FLICK;
                }
                break;

            case DONE:
                return Result.SHOT;
            case FAIL:
                return Result.FAIL;
            case NO_BALL:
                return Result.NO_BALL;
        }

        tele.addLine("--- DEPOSIT ---")
                .addData("S", s::name)
                .addData("BallVerified", "%b", ballVerified)
                .addData("FrontHasBall", "%b", frontHasBall)
                .addData("EmptyConfirm", "%d", emptyConfirmCount)
                .addData("SawRpmShot", "%b", sawRpmShot)
                .addData("Refires", "%d", refires)
                .addData("Jiggles", "%d", jiggles);
        return Result.BUSY;
    }
}
