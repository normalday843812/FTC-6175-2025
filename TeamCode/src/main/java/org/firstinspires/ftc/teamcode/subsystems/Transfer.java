package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.TransferConfig.FLICK_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.MAX;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.MIN;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.RESET_TIME_S;
import static org.firstinspires.ftc.teamcode.config.TransferConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Transfer {
    private enum FlickState {IDLE, FLICK_UP, FLICK_DOWN}

    private final Servo transferServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    private FlickState state = FlickState.IDLE;
    private final Timer flickTimer = new Timer();

    public Transfer(Servo transferServo, GamepadMap map, OpMode opmode) {
        this.transferServo = transferServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
        this.transferServo.setPosition(MIN);
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
                transferServo.setPosition(MAX);
                if (flickTimer.getElapsedTimeSeconds() >= FLICK_TIME_S) {
                    state = FlickState.FLICK_DOWN;
                    flickTimer.resetTimer();
                }
                break;
            case FLICK_DOWN:
                transferServo.setPosition(MIN);
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
                .addData("Pos", "%.2f", transferServo.getPosition());
    }
}
