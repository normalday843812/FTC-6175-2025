package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.STEP;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Spindexer {
    private final Servo spindexerServo;
    private final GamepadMap map;
    private final TelemetryHelper tele;

    public Spindexer(Servo spindexerServo, GamepadMap map, OpMode opmode) {
        this.spindexerServo = spindexerServo;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void operate() {
        if (map.spindexerForward && spindexerServo.getPosition() < ((int) (1 / STEP)) * STEP) {
            spindexerServo.setPosition(spindexerServo.getPosition() + STEP);
        }
        if (map.spindexerBackward) {
            spindexerServo.setPosition(spindexerServo.getPosition() - STEP);
        }

        addTelemetry();
    }

    private void addTelemetry() {
        tele.addLine("=== SPINDEXER ===")
                .addData("Spindexer Position", "%.2f", spindexerServo.getPosition());
    }
}
