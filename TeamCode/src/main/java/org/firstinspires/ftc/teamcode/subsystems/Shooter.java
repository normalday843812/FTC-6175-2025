package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MAX_RPM;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.MOTOR_1_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.RPM_AT_SHOT;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TPR_MOTOR;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TPR_OUTPUT;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TRIGGER_SCALE_DOWN;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TRIGGER_SCALE_UP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Shooter {
    private final GamepadMap map;
    private final DcMotorEx motor;
    private final DcMotorEx motor1;
    // Telemetry
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;
    private double targetRpm = 0.0;
    private double shotCount = 0;
    private boolean shot = false;
    private double idleRpmMin = ShooterConfig.IDLE_RPM;

    public Shooter(DcMotorEx motor, DcMotorEx motor1, GamepadMap map, OpMode opmode) {
        this.motor = motor;
        this.motor1 = motor1;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void setIdleRpm(double idle) {
        this.idleRpmMin = idle;
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
    }

    public void operate() {
        if (getMotorRPM() < RPM_AT_SHOT && !shot) {
            shotCount++;
            shot = true;
        }
        if (mode == SubsystemMode.MANUAL) {
            operateManual();
        } else {
            // AUTO: hold target RPM set via setAutoRpm
            setShooterOutputRpm(targetRpm);
        }
        addTelemetry();
        if (getMotorRPM() > RPM_AT_SHOT) {
            shot = false;
        }
    }

    private void operateManual() { // Manual uses motor RPM
        if (map == null) return;

        if (targetRpm < MAX_RPM) {
            targetRpm += map.shooterUp * TRIGGER_SCALE_UP * 3;
        }
        if (targetRpm > 0) {
            targetRpm -= map.shooterDown * TRIGGER_SCALE_DOWN * 3;
        }
        if (targetRpm < idleRpmMin) {
            targetRpm = idleRpmMin;
        }

        setShooterMotorRpm(targetRpm);
    }

    public void setAutoRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
    }

    public double getMotorRPM() {
        double tps = motor.getVelocity();
        if (MOTOR_1_ENABLED) tps = (tps + motor1.getVelocity()) / 2;
        return tps * 60.0 / TPR_MOTOR;
    }

    public double getOutputRPM() {
        double tps = motor.getVelocity();
        if (MOTOR_1_ENABLED) tps = (tps + motor1.getVelocity()) / 2;
        return tps * 60.0 / TPR_OUTPUT;
    }

    public boolean isAtTarget(double band) {
        if (mode == SubsystemMode.MANUAL) {
            return Math.abs(getMotorRPM() - targetRpm) <= band;
        }
        return Math.abs(getOutputRPM() - targetRpm) <= band;
    }

    private void setShooterOutputRpm(double outputRpm) {
        double tps = outputRpm * TPR_OUTPUT / 60.0;
        motor.setVelocity(tps);
        if (MOTOR_1_ENABLED) motor1.setVelocity(tps);
    }

    private void setShooterMotorRpm(double outputRpm) {
        double tps = outputRpm * TPR_MOTOR / 60.0;
        motor.setVelocity(tps);
        if (MOTOR_1_ENABLED) motor1.setVelocity(tps);
    }

    public double getMotorCurrent(CurrentUnit currentUnit) {
        if (MOTOR_1_ENABLED) return motor.getCurrent(currentUnit) + motor1.getCurrent(currentUnit);
        return motor.getCurrent(currentUnit);
    }

    private void addTelemetry() {
        double tps0 = motor.getVelocity();
        double tps1 = motor1.getVelocity();
        tele.addLine("=== SHOOTER ===")
                .addData("Mode", mode::name)
                .addData("Target RPM", "%.0f", targetRpm)
                .addData("Output RPM (0)", "%.0f", tps0 * 60.0 / TPR_OUTPUT)
                .addData("Motor RPM (0)", "%.0f", tps0 * 60.0 / TPR_MOTOR)
                .addData("Output RPM (1)", "%.0f", tps1 * 60.0 / TPR_OUTPUT)
                .addData("Motor RPM (1)", "%.0f", tps1 * 60.0 / TPR_MOTOR)
                .addData("Shot Count", "%.0f", shotCount);
    }
}
