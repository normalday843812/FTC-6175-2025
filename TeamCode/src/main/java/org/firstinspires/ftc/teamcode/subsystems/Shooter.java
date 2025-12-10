package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MAX_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MOTOR1_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.SHOT_ARM_AT_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.SHOT_ARM_DWELL_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.SHOT_DROP_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.SHOT_DROP_WINDOW_MS;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TPR_MOTOR;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Shooter {
    private final DcMotorEx motor;
    private final DcMotorEx motor1;
    // Telemetry
    private final TelemetryHelper tele;
    private double targetRpm = 0.0;

    private boolean armed = false;
    private boolean shotPulse = false;
    private double rpmMaxSinceArm = 0.0;
    private long armStartMs = 0;
    private long lastUpdateMs = 0;
    private int shotCount = 0;

    public Shooter(DcMotorEx motor, DcMotorEx motor1, OpMode opmode) {
        this.motor = motor;
        this.motor1 = motor1;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setIdleRpm(double idle) {
        this.idleRpmMin = idle;
    }

    public void start() {
        resetShotLogic();
    }

    public void operate() {
        shotPulse = false;
        updateShotLogic();

        setShooterMotorRpm(targetRpm);
        addTelemetry();
    }

    public void setAutoRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
    }

    public double getMotorRPM() {
        double tps = motor.getVelocity();
        if (MOTOR1_ENABLED) tps = (tps + motor1.getVelocity()) / 2;
        return tps * 60.0 / TPR_MOTOR;
    }

    public boolean isAtTarget(double band) {
        return Math.abs(getMotorRPM() - targetRpm) <= band;
    }

    public boolean shotOccurred() {
        return shotPulse;
    }

    private void setShooterMotorRpm(double motorRpm) {
        double tps = motorRpm * TPR_MOTOR / 60.0;
        motor.setVelocity(tps);
        motor1.setVelocity(tps);
    }

    public void resetShotLogic() {
        armed = false;
        shotPulse = false;
        rpmMaxSinceArm = 0.0;
        armStartMs = 0;
        lastUpdateMs = now();
        shotCount = 0;
    }

    private void updateShotLogic() {
        double rpm = getMotorRPM();
        long n = now();

        if (!armed) {
            if (rpm >= SHOT_ARM_AT_RPM) {
                if (armStartMs == 0) armStartMs = n;
                if (n - armStartMs >= SHOT_ARM_DWELL_MS) {
                    armed = true;
                    rpmMaxSinceArm = rpm;
                }
            } else {
                armStartMs = 0;
            }
        } else {
            if (rpm > rpmMaxSinceArm) rpmMaxSinceArm = rpm;
            boolean drop = (rpmMaxSinceArm - rpm) >= SHOT_DROP_RPM;
            boolean inWindow = (n - lastUpdateMs) <= SHOT_DROP_WINDOW_MS || (n - armStartMs) <= 2000;
            if (drop && inWindow) {
                shotPulse = true;
                shotCount++;
                armed = false;
                rpmMaxSinceArm = 0.0;
                armStartMs = 0;
            }
        }

        lastUpdateMs = n;
    }

    private static long now() {
        return System.nanoTime() / 1_000_000L;
    }

    private void addTelemetry() {
        double tps0 = motor.getVelocity();
        double tps1 = motor1.getVelocity();
        tele.addLine("=== SHOOTER ===")
                .addData("Target RPM", "%.0f", targetRpm)
                .addData("Motor RPM (0)", "%.0f", tps0 * 60.0 / TPR_MOTOR)
                .addData("Motor RPM (1)", "%.0f", tps1 * 60.0 / TPR_MOTOR)
                .addData("Shot Count", "%d", shotCount);
    }
}