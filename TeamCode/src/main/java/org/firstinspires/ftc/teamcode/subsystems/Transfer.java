package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.FLICK_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MAX;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_1_MIN;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.RESET_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_2_MAX;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TRANSFER_2_MIN;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Transfer {
    private enum FlickState {IDLE, FLICK_UP, FLICK_DOWN}

    private final Servo transferServo1, transferServo2;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    private FlickState state = FlickState.IDLE;
    private final Timer flickTimer = new Timer();

    public Transfer(Servo transferServo1, Servo transferServo2, GamepadMap map, OpMode opmode) {
        this.transferServo1 = transferServo1;
        this.transferServo2 = transferServo2;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
        this.transferServo1.setPosition(TRANSFER_1_MIN);
        this.transferServo2.setPosition(TRANSFER_2_MIN);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL && map != null) {
            if (map.transferButton) {
                flick();
            }
        }

        switch (state) {
            case IDLE:
                // nothing
                break;
            case FLICK_UP:
                transferServo1.setPosition(TRANSFER_1_MAX);
                transferServo2.setPosition(TRANSFER_2_MAX);
                if (flickTimer.getElapsedTimeSeconds() >= FLICK_TIME_S) {
                    state = FlickState.FLICK_DOWN;
                    flickTimer.resetTimer();
                }
                break;
            case FLICK_DOWN:
                transferServo1.setPosition(TRANSFER_1_MIN);
                transferServo2.setPosition(TRANSFER_2_MIN);
                if (flickTimer.getElapsedTimeSeconds() >= RESET_TIME_S) {
                    state = FlickState.IDLE;
                }
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
                .addData("State", state::name)
                .addData("Pos1", "%.2f", transferServo1.getPosition())
                .addData("Pos2", "%.2f", transferServo2.getPosition());
    }
}
