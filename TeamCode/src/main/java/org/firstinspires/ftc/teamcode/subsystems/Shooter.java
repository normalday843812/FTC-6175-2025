package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MAX_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TPR_MOTOR;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TPR_OUTPUT;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TRIGGER_SCALE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Shooter {
    private final GamepadMap map;
    private final DcMotorEx motor;
    // Telemetry
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private static double targetRpm = 0.0;

    public Shooter(DcMotorEx motor, GamepadMap map, OpMode opmode) {
        this.motor = motor;
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
            operateManual();
        } else {
            // AUTO: hold target RPM set via setAutoRpm
            motor.setVelocity(targetRpm * TPR_OUTPUT / 60.0);
        }
        addTelemetry();
    }

    private void operateManual() {
        if (map == null) return;

        if (targetRpm < MAX_RPM) {
            targetRpm += map.shooterUp * TRIGGER_SCALE;
        }
        targetRpm -= map.shooterDown * TRIGGER_SCALE;

        motor.setVelocity(targetRpm * TPR_OUTPUT / 60.0);
    }

    public void setAutoRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(ShooterConfig.MAX_RPM, rpm));
    }

    public double getMotorRPM() {
        double tps = motor.getVelocity();
        return tps * 60.0 / TPR_MOTOR;
    }

    public double getOutputRPM() {
        double tps = motor.getVelocity();
        return tps * 60.0 / TPR_OUTPUT;
    }

    public boolean isAtTarget(double band) {
        return Math.abs(getOutputRPM() - targetRpm) <= band;
    }

    private void addTelemetry() {
        double tps = motor.getVelocity();
        tele.addLine("=== SHOOTER ===")
                .addData("Mode", mode::name)
                .addData("Target RPM", "%.0f", targetRpm)
                .addData("Output RPM", "%.0f", tps * 60.0 / TPR_OUTPUT)
                .addData("Motor RPM", "%.0f", tps * 60.0 / TPR_MOTOR);
    }
}
