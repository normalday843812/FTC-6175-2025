package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Shooter {
    private final OpMode opmode;
    private final GamepadMap map;
    private final DcMotorEx motor;
    private double targetRpm = 0.0;

    public Shooter(DcMotorEx motor, GamepadMap map, OpMode opmode) {
        this.motor=motor;
        this.map=map;
        this.opmode=opmode;
    }

    public void operate() {
        if (map.shooterTrigger > ShooterConfig.TRIGGER_DB) targetRpm = map.shooterTrigger * ShooterConfig.MAX_RPM;
        setRpm(targetRpm);

        double tps = motor.getVelocity();
        double motorRPM = tps * 60.0 / ShooterConfig.TPR_MOTOR;
        double outputRPM = tps * 60.0 / ShooterConfig.TPR_OUTPUT;

        new TelemetryHelper(opmode, TELEMETRY_ENABLED)
                .addLine("--- SHOOTER ---")
                .addData("Target RPM:", "%.0f", targetRpm)
                .addData("Output RPM:", "%.0f", outputRPM)
                .addData("Motor RPM:", "%.0f", motorRPM)
                .addData("err tps:", "%.1f", targetRpm * ShooterConfig.TPR_OUTPUT / 60.0 - tps);
    }

    public void setRpm(double rpm){
        rpm = Math.max(0.0, Math.min(ShooterConfig.MAX_RPM, rpm));
        motor.setVelocity(rpm * ShooterConfig.TPR_OUTPUT / 60.0);
    }
}
