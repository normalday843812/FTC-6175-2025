package org.firstinspires.ftc.teamcode.managers;

import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.AT_RPM_WAIT_TIMEOUT_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.INDEX_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DELTA_DOWN;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DELTA_UP;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_DWELL_S;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.JIGGLE_MAX;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.REFIRE_MAX;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.RPM_VERIFY_DROP;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.TARGET_RPM_BAND;
import static org.firstinspires.ftc.teamcode.config.AutoUnifiedConfig.VERIFY_WINDOW_S;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class DepositController {
    public enum Result {BUSY, SHOT, FAIL}

    private enum S {SPINUP, WAIT_INDEX, WAIT_FLICK, VERIFY, JIGGLE, DONE, FAIL}

    private final Shooter shooter;
    private final Transfer transfer;
    private final Spindexer spx;
    private final TelemetryHelper tele;

    private final Timer tState = new Timer();
    private final Timer tVerify = new Timer();

    private S s = S.SPINUP;
    private int refires = 0, jiggles = 0;

    public DepositController(Shooter shooter, Transfer transfer, Spindexer spx, TelemetryHelper tele) {
        this.shooter = shooter;
        this.transfer = transfer;
        this.spx = spx;
        this.tele = tele;
        tState.resetTimer();
    }

    public void reset() {
        s = S.SPINUP;
        refires = 0;
        jiggles = 0;
        tState.resetTimer();
        spx.stopJiggle();
    }

    public Result update(double targetRpm) {
        shooter.setAutoRpm(targetRpm);
        shooter.operate();
        transfer.operate();

        switch (s) {
            case SPINUP:
                if (shooter.isAtTarget(TARGET_RPM_BAND) || tState.getElapsedTimeSeconds() >= AT_RPM_WAIT_TIMEOUT_S) {
                    if (transfer.isLeverRaised()) {
                        transfer.lowerLever();
                        break;
                    }
                    spx.stepForward();
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
                    s = S.VERIFY;
                }
                break;

            case VERIFY:
                if (shooter.shotOccurred()) {
                    s = S.DONE;
                } else if (tVerify.getElapsedTimeSeconds() >= VERIFY_WINDOW_S) {
                    if (refires < REFIRE_MAX) {
                        refires++;
                        transfer.flick();
                        s = S.WAIT_FLICK;
                    } else if (jiggles < JIGGLE_MAX) {
                        jiggles++;
                        spx.startJiggle(JIGGLE_DELTA_UP, JIGGLE_DELTA_DOWN, JIGGLE_DWELL_S);
                        s = S.JIGGLE;
                    } else {
                        s = S.FAIL;
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
        }

        tele.addLine("--- DEPOSIT ---")
                .addData("S", s::name)
                .addData("Refires", "%d", refires)
                .addData("Jiggles", "%d", jiggles)
                .addData("OutRPM", "%.0f", shooter.getOutputRPM())
                .addData("MotorRPM", "%.0f", shooter.getMotorRPM());
        return Result.BUSY;
    }
}