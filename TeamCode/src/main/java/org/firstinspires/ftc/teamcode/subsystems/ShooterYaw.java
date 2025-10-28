package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class ShooterYaw {
    private final GamepadMap map;
    private DcMotorEx shooterYawMotor;
    private final TelemetryHelper tele;
    private SubsystemMode mode = SubsystemMode.MANUAL;

    private int position = 0;

    public ShooterYaw(DcMotorEx shooterYawMotor, GamepadMap map, OpMode opmode) {
        this.shooterYawMotor = shooterYawMotor;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
        resetShooterYaw();
    }

    public void operate() {
        position = shooterYawMotor.getCurrentPosition();
        shooterYawMotor.setTargetPosition(position + ((int) (map.shooterYaw * 5)));
        addTelemetry();
    }

    public void resetShooterYaw() {
        shooterYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void addTelemetry() {
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Position", "%.0f", (double) position);
    }
}
