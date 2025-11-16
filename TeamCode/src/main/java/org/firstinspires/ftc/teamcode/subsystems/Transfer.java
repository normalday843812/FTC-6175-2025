package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.FLICK_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.RESET_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.SHOOTING_MODE_DURATION_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MAX;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MIN;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MIN_SHOOTING;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Transfer {
    private enum FlickState {IDLE, FLICK_UP, FLICK_DOWN}

    public enum CrState {OFF, FORWARD, REVERSE}

    private final Servo transferServo1;
    private final CRServo transferCrServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    private FlickState state = FlickState.IDLE;
    private CrState crState = CrState.FORWARD;
    private final Timer flickTimer = new Timer();
    private final Timer shootingModeTimer = new Timer();
    private boolean shootingMode = false;
    private boolean manualCrControl = false;

    public Transfer(Servo transferServo1, CRServo transferCrServo, GamepadMap map, OpMode opmode) {
        this.transferServo1 = transferServo1;
        this.transferCrServo = transferCrServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        // start retracted
        this.transferServo1.setPosition(TRANSFER_1_MIN);
        // start CR servo in forward
        this.transferCrServo.setPower(1.0);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
    }

    public void operate() {
        // driver inputs
        if (mode == SubsystemMode.MANUAL && map != null) {
            // A button puts transfer in shooting position for 2 seconds
            if (map.shootingModeToggle) {
                shootingMode = true;
                shootingModeTimer.resetTimer();
                if (state == FlickState.IDLE) {
                    transferServo1.setPosition(TRANSFER_1_MIN_SHOOTING);
                }
            }

            // flick is the old behavior
            if (map.transferButton) {
                crState = CrState.FORWARD;
                flick();
            }

            // new CR servo controls on dpad right/left
            if (map.transferCrForward) {
                // tap to toggle forward
                crState = (crState == CrState.FORWARD) ? CrState.OFF : CrState.FORWARD;
                manualCrControl = (crState != CrState.OFF);
            }
            if (map.transferCrReverse) {
                // tap to toggle reverse
                crState = (crState == CrState.REVERSE) ? CrState.OFF : CrState.REVERSE;
                manualCrControl = (crState != CrState.OFF);
            }
        }

        // determine the min position based on shooting mode
        double minPosition = shootingMode ? TRANSFER_1_MIN_SHOOTING : TRANSFER_1_MIN;

        // check if shooting mode timer has expired (2 seconds)
        if (shootingMode && shootingModeTimer.getElapsedTimeSeconds() >= SHOOTING_MODE_DURATION_S) {
            shootingMode = false;
            if (state == FlickState.IDLE) {
                transferServo1.setPosition(TRANSFER_1_MIN);
            }
        }

        // flick FSM (positional servo)
        switch (state) {
            case IDLE:
                // only set position in IDLE if we just toggled shooting mode
                // otherwise position was already set and should not be overwritten
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
        }

        // CR servo: always reverse when in shooting position (unless manual override)
        if (!manualCrControl) {
            if (shootingMode) {
                crState = CrState.REVERSE;
            } else {
                crState = CrState.FORWARD;
            }
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
            shootingMode = true;
            shootingModeTimer.resetTimer();
            transferServo1.setPosition(TRANSFER_1_MIN_SHOOTING);
        }
    }

    public void lowerLever() {
        // Only lower if shooting mode timer has expired
        if (!shootingMode) {
            if (state == FlickState.IDLE) {
                transferServo1.setPosition(TRANSFER_1_MIN);
            }
        }
    }

    public void runTransfer(CrState state) {
        if (!manualCrControl) {
            crState = state;
        }
    }

    public boolean isLeverRaised() {
        return shootingMode || (state != FlickState.IDLE);
    }

    public boolean isIdle() {
        return state == FlickState.IDLE;
    }

    private void addTelemetry() {
        tele.addLine("=== TRANSFER ===")
                .addData("Mode", mode::name)
                .addData("Flick State", state::name)
                .addData("CR State", crState::name)
                .addData("Shooting Mode", () -> shootingMode);
    }
}