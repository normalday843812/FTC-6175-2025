package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.FLICK_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.RESET_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MAX;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MIN;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_LEVER_UP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Transfer {
    private enum FlickState {IDLE, FLICK_UP, FLICK_DOWN, HOLD_UP}

    public enum CrState {OFF, FORWARD, REVERSE}

    private final Servo transferServo1;
    private final CRServo transferCrServo;
    private final TelemetryHelper tele;

    private FlickState state = FlickState.IDLE;
    private CrState crState = CrState.OFF;
    private final Timer flickTimer = new Timer();
    private boolean leverRaised = false;

    public Transfer(Servo transferServo1, CRServo transferCrServo, OpMode opmode) {
        this.transferServo1 = transferServo1;
        this.transferCrServo = transferCrServo;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        this.transferServo1.setPosition(TRANSFER_1_MIN);
        this.transferCrServo.setPower(0.0);
    }

    public void start() {
        state = FlickState.IDLE;
        crState = CrState.OFF;
        leverRaised = false;
        transferServo1.setPosition(TRANSFER_1_MIN);
        transferCrServo.setPower(0.0);
    }

    public void operate() {
        double minPosition = leverRaised ? TRANSFER_1_LEVER_UP : TRANSFER_1_MIN;

        switch (state) {
            case IDLE:
                transferServo1.setPosition(minPosition);
                break;
            case FLICK_UP:
                transferServo1.setPosition(TRANSFER_1_MAX);
                if (flickTimer.getElapsedTimeSeconds() >= FLICK_TIME_S) {
                    state = FlickState.FLICK_DOWN;
                    flickTimer.resetTimer();
                }
                break;
            case FLICK_DOWN:
                transferServo1.setPosition(minPosition);
                if (flickTimer.getElapsedTimeSeconds() >= RESET_TIME_S) {
                    state = FlickState.IDLE;
                }
                break;
            case HOLD_UP:
                transferServo1.setPosition(TRANSFER_1_MAX);
                break;
        }

        switch (crState) {
            case OFF:
                transferCrServo.setPower(0.0);
                break;
            case FORWARD:
                transferCrServo.setPower(1.0);
                break;
            case REVERSE:
                transferCrServo.setPower(-1.0);
                break;
        }

        addTelemetry();
    }

    public void flick() {
        if (state == FlickState.IDLE) {
            state = FlickState.FLICK_UP;
            flickTimer.resetTimer();
        }
    }

    public void raiseLever() {
        if (state == FlickState.IDLE) {
            leverRaised = true;
        }
    }

    public void lowerLever() {
        leverRaised = false;
    }

    public void runTransfer(CrState state) {
        crState = state;
    }

    public boolean isLeverRaised() {
        return leverRaised || (state != FlickState.IDLE);
    }

    public boolean isIdle() {
        return state == FlickState.IDLE;
    }

    public void holdUp() {
        state = FlickState.HOLD_UP;
    }

    public void releaseHold() {
        state = FlickState.FLICK_DOWN;
        flickTimer.resetTimer();

    }

    private void addTelemetry() {
        tele.addLine("=== TRANSFER ===")
                .addData("Flick State", state::name)
                .addData("CR State", crState::name)
                .addData("Lever Up", () -> leverRaised);
    }
}
