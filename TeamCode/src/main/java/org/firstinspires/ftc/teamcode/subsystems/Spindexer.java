package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.BIAS;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.SLOTS;
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
        spindexerServo.setPosition(BIAS);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        spindexerServo.setPosition(BIAS);
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        spindexerServo.setPosition(BIAS);
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL) {
            if (map != null) {
                if (map.spindexerForward) {
                    stepForward();
                }
                if (map.spindexerBackward) {
                    stepBackward();
                }
            }
        }
        addTelemetry();
    }

    public int getSlots() {
        return SLOTS > 0 ? SLOTS : (int) Math.max(1, Math.round(1.0 / STEP));
    }

    public int getCurrentSlot() {
        double pos = getPosition() - BIAS;
        if (pos < 0) pos += 1.0;
        int slot = (int) Math.round(pos / STEP);
        return ((slot % getSlots()) + getSlots()) % getSlots();
    }

    public void setSlot(int idx) {
        int slots = getSlots();
        int k = ((idx % slots) + slots) % slots;
        double target = BIAS + k * STEP;
        while (target > 1.0) target -= 1.0;
        while (target < 0.0) target += 1.0;
        setAbsolute(target);
    }

    public void stepSlots(int delta) {
        setSlot(getCurrentSlot() + delta);
    }

    public void indexToColor(IntakeColorSensor cs, boolean targetPurple, int maxStepsToScan, long timeoutMs) {
        long deadline = System.currentTimeMillis() + Math.max(0, timeoutMs);
        int steps = 0;
        while (steps < Math.max(1, maxStepsToScan) && System.currentTimeMillis() < deadline) {
            if (targetPurple ? cs.isConsistentlyPurple() : cs.isConsistentlyGreen()) {
                return;
            }
            stepSlots(1);
            long dwell = System.currentTimeMillis() + jiggleDwellMs();
            while (System.currentTimeMillis() < dwell) {
                // let sensor update
            }
            steps++;
        }
    }

    private long jiggleDwellMs() {
        try {
            return (long) (org.firstinspires.ftc.teamcode.config.AutoDepositConfig.JIGGLE_DWELL_S * 1000);
        } catch (Throwable t) {
            return 120;
        }
    }

    public void stepForward() {
        stepSlots(1);
    }

    public void stepBackward() {
        stepSlots(-1);
    }

    public void setAbsolute(double pos) {
        spindexerServo.setPosition(Math.max(0.0, Math.min(1.0, pos)));
    }

    public double getPosition() {
        return spindexerServo.getPosition();
    }

    public Spindexer.JigglePlan makeJigglePlan(double deltaUp, double deltaDown) {
        double base = getPosition();
        double up = Math.min(1.0, base + deltaUp);
        double down = Math.max(0.0, base - deltaDown);
        return new Spindexer.JigglePlan(base, up, down);
    }

    public static class JigglePlan {
        public final double base;
        public final double up;
        public final double down;

        public JigglePlan(double base, double up, double down) {
            this.base = base;
            this.up = up;
            this.down = down;
        }
    }

    private void addTelemetry() {
        tele.addLine("=== SPINDEXER ===")
                .addData("Mode", mode::name)
                .addData("Pos", "%.2f", spindexerServo.getPosition())
                .addData("Slot", "%d/%d", getCurrentSlot(), getSlots());
    }
}