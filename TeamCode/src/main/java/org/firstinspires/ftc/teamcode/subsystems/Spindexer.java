package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.BIAS;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.SLOTS;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.STEP;
import static org.firstinspires.ftc.teamcode.config.SpindexerConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;
import org.firstinspires.ftc.teamcode.util.Timer;

public class Spindexer {
    private static final long SETTLE_TIME_MS = 300;

    private final Servo spindexerServo;
    private final TelemetryHelper tele;
    private int commandedSlot = 0;
    private long lastCommandMs = 0;

    // Jiggle state
    private boolean jiggleActive = false;
    private double jUp, jDown, jDwellS;
    private int jPhase = 0;
    private final Timer jTimer = new Timer();
    private double lastJiggleBase = 0.0;

    public Spindexer(Servo spindexerServo, OpMode opmode) {
        this.spindexerServo = spindexerServo;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void start() {
        spindexerServo.setPosition(BIAS);
        commandedSlot = 0;
    }

    public void operate() {
        if (jiggleActive) {
            switch (jPhase) {
                case 0:
                    spindexerServo.setPosition(jUp);
                    if (jTimer.getElapsedTimeSeconds() >= jDwellS) {
                        jPhase = 1;
                        jTimer.resetTimer();
                    }
                    break;
                case 1:
                    spindexerServo.setPosition(jDown);
                    if (jTimer.getElapsedTimeSeconds() >= jDwellS) {
                        jPhase = 2;
                        jTimer.resetTimer();
                    }
                    break;
                case 2:
                    spindexerServo.setPosition(lastJiggleBase);
                    if (jTimer.getElapsedTimeSeconds() >= jDwellS) {
                        jiggleActive = false;
                    }
                    break;
            }
        }

        addTelemetry();
    }

    public int getSlots() {
        if (SLOTS > 0) return SLOTS;
        if (STEP <= 0.0) return 1;
        return (int) Math.max(1, Math.round(1.0 / STEP));
    }

    private double effectiveStep() {
        int slots = getSlots();
        if (STEP > 0.0) return STEP;
        if (slots <= 0) return 1.0;
        return 1.0 / slots;
    }

    public int getCurrentSlot() {
        double step = effectiveStep();
        double pos = getPosition() - BIAS;
        if (pos < 0) pos += 1.0;
        int slot = (int) Math.round(pos / step);
        return ((slot % getSlots()) + getSlots()) % getSlots();
    }

    public boolean isMoving() {
        return jiggleActive || (System.currentTimeMillis() - lastCommandMs < SETTLE_TIME_MS);
    }

    public boolean isSettled() {
        return !isMoving();
    }

    public void setSlot(int idx) {
        int slots = getSlots();
        int k = ((idx % slots) + slots) % slots;

        commandedSlot = k;
        lastCommandMs = System.currentTimeMillis();

        double target = BIAS + k * effectiveStep();
        while (target > 1.0) target -= 1.0;
        while (target < 0.0) target += 1.0;
        setAbsolute(target);
    }

    public int getCommandedSlot() {
        return commandedSlot;
    }

    public void stepSlots(int delta) {
        setSlot(getCurrentSlot() + delta);
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

    public void startJiggle(double deltaUp, double deltaDown, double dwellS) {
        double base = getPosition();
        double up = Math.min(1.0, base + deltaUp);
        double down = Math.max(0.0, base - deltaDown);
        lastJiggleBase = base;
        jUp = up;
        jDown = down;
        jDwellS = dwellS;
        jPhase = 0;
        jiggleActive = true;
        jTimer.resetTimer();
    }

    public boolean updateJiggle() {
        return !jiggleActive;
    }

    public void stopJiggle() {
        jiggleActive = false;
    }

    private void addTelemetry() {
        tele.addLine("=== SPINDEXER ===")
                .addData("Pos", "%.2f", spindexerServo.getPosition())
                .addData("Slot", "%d", getCurrentSlot())
                .addData("CmdSlot", "%d", commandedSlot)
                .addData("Moving", "%b", isMoving())
                .addData("JiggleActive", "%b", jiggleActive)
                .addData("JigglePhase", "%d", jPhase)
                .addData("LastJiggleBase", "%.2f", lastJiggleBase);
    }
}
