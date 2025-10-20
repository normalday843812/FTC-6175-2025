package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.STEP;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Spindexer {
    private final Servo spindexerServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    public Spindexer(Servo spindexerServo, GamepadMap map, OpMode opmode) {
        this.spindexerServo = spindexerServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL) {
            if (map != null) {
                if (map.spindexerForward && spindexerServo.getPosition() < ((int)(1/STEP))*STEP) {
                    spindexerServo.setPosition(spindexerServo.getPosition() + STEP);
                }
                if (map.spindexerBackward) {
                    spindexerServo.setPosition(spindexerServo.getPosition() - STEP);
                }
            }
        }
        addTelemetry();
    }

    public void stepForward() {
        double pos = spindexerServo.getPosition();
        spindexerServo.setPosition(Math.max(0.0, Math.min(1.0, pos + STEP)));
    }

    public void stepBackward() {
        double pos = spindexerServo.getPosition();
        spindexerServo.setPosition(Math.max(0.0, Math.min(1.0, pos - STEP)));
    }

    public void setAbsolute(double pos) {
        spindexerServo.setPosition(Math.max(0.0, Math.min(1.0, pos)));
    }

    private void addTelemetry() {
        tele.addLine("=== SPINDEXER ===")
                .addData("Mode", mode::name)
                .addData("Position", "%.2f", spindexerServo.getPosition());
    }
}
