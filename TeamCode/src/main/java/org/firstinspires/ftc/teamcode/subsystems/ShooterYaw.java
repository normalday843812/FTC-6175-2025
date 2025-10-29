package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.CONTROL_LEVEL;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.config.ShooterYawConfig.YAW_POWER;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class ShooterYaw {
    private final GamepadMap map;
    private final DcMotorEx shooterYawMotor;
    private final TelemetryHelper tele;
    private SubsystemMode mode = SubsystemMode.MANUAL;

    private int targetPosition = 0;
    private int position = 0;

    public ShooterYaw(DcMotorEx shooterYawMotor, GamepadMap map, OpMode opmode) {
        this.shooterYawMotor = shooterYawMotor;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, TELEMETRY_ENABLED);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
    }

    public void operate() {
        position = shooterYawMotor.getCurrentPosition();

        if ((map.shooterYaw < 0 && targetPosition > MIN_POSITION) ||
                (map.shooterYaw > 0 && targetPosition < MAX_POSITION)) {

            targetPosition += (int) (map.shooterYaw * CONTROL_LEVEL);
        }

        if (map.resetShooterYaw) resetShooterYaw();

        if (targetPosition < MIN_POSITION) targetPosition = MIN_POSITION;
        if (targetPosition > MAX_POSITION) targetPosition = MAX_POSITION;

        if (shooterYawMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        shooterYawMotor.setTargetPosition(targetPosition);
        shooterYawMotor.setPower(YAW_POWER);

        addTelemetry();
    }

    public void resetShooterYaw() {
        shooterYawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetPosition = 0;
        position = 0;

        shooterYawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterYawMotor.setTargetPosition(targetPosition);
        shooterYawMotor.setPower(YAW_POWER);
    }

    public void addTelemetry() {
        tele.addLine("=== SHOOTER YAW ===")
                .addData("Current Pos", "%d", position)
                .addData("Target Pos", "%d", targetPosition)
                .addData("Mode", "%s", mode.toString());
    }
}
