package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.FLICK_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.RESET_TIME_S;
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

    private enum CrState {OFF, FORWARD, REVERSE}

    private final Servo transferServo1;
    private final CRServo transferCrServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    private FlickState state = FlickState.IDLE;
    private CrState crState = CrState.OFF;
    private final Timer flickTimer = new Timer();
    private boolean shootingMode = false;

    public Transfer(Servo transferServo1, CRServo transferCrServo, GamepadMap map, OpMode opmode) {
        this.transferServo1 = transferServo1;
        this.transferCrServo = transferCrServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);

        // start retracted
        this.transferServo1.setPosition(TRANSFER_1_MIN);
        // make sure CR servo is stopped
        this.transferCrServo.setPower(0.0);
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
            // toggle shooting mode with A button
            if (map.shootingModeToggle) {
                // if already in shooting mode, reverse the intake CR servo
                if (shootingMode) {
                    crState = (crState == CrState.REVERSE) ? CrState.OFF : CrState.REVERSE;
                }
                
                shootingMode = !shootingMode;
                // immediately move to the new position when toggling modes and idle
                if (state == FlickState.IDLE) {
                    double newPosition = shootingMode ? TRANSFER_1_MIN_SHOOTING : TRANSFER_1_MIN;
                    transferServo1.setPosition(newPosition);
                }
            }

            // flick is the old behavior
            if (map.transferButton) {
                flick();
            }

            // new CR servo controls on dpad right/left
            if (map.transferCrForward) {
                // tap to toggle forward
                crState = (crState == CrState.FORWARD) ? CrState.OFF : CrState.FORWARD;
            }
            if (map.transferCrReverse) {
                // tap to toggle reverse
                crState = (crState == CrState.REVERSE) ? CrState.OFF : CrState.REVERSE;
            }
        }

        // determine the min position based on shooting mode
        double minPosition = shootingMode ? TRANSFER_1_MIN_SHOOTING : TRANSFER_1_MIN;

        // flick FSM (positional servo)
        switch (state) {
            case IDLE:
                // maintain the correct position while idle
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
