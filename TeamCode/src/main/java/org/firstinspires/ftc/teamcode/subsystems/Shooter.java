package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.GlobalConfig.FALLBACK_MODE;
import static org.firstinspires.ftc.teamcode.config.ShooterConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Shooter {
    private final GamepadMap map;
    private final DcMotorEx motor;
    // Telemetry
    private final TelemetryHelper tele;


    private static double targetRpm = 0.0;

    public Shooter(DcMotorEx motor, GamepadMap map, OpMode opmode) {
        this.motor = motor;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void operate() {
        if (FALLBACK_MODE) {
            operateFallback();
            return;
        }

        double desired = (map.shooterTrigger > ShooterConfig.TRIGGER_DB)
                ? map.shooterTrigger * ShooterConfig.MAX_RPM
                : 0.0;

        setRpm(desired);
        addTelemetry();
    }

    private void operateFallback() {
        motor.setPower(map.shooterTrigger);
        tele.addLine("--- SHOOTER FALLBACK ---")
                .addData("Power:", "%.2f", motor.getPower());
    }

    public void setRpm(double rpm) {
        targetRpm = Math.max(0.0, Math.min(ShooterConfig.MAX_RPM, rpm));
        motor.setVelocity(targetRpm * ShooterConfig.TPR_OUTPUT / 60.0);
    }

    public double getMotorRPM() {
        return motor.getVelocity() * 60.0 / ShooterConfig.TPR_MOTOR;
    }

    private void addTelemetry() {
        double tps = motor.getVelocity();
        double motorRPM = tps * 60.0 / ShooterConfig.TPR_MOTOR;
        double outputRPM = tps * 60.0 / ShooterConfig.TPR_OUTPUT;

        tele.addLine("--- SHOOTER ---")
                .addData("Target RPM:", "%.0f", targetRpm)
                .addData("Output RPM:", "%.0f", outputRPM)
                .addData("Motor RPM:", "%.0f", motorRPM)
                .addData("err tps:", "%.1f", targetRpm * ShooterConfig.TPR_OUTPUT / 60.0 - tps);
    }
}
