package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.TPRShooter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;

public class Shooter {
    OpMode opmode;

    // Telemetry
    public static boolean TELEMETRY_ENABLED = true;

    GamepadMap map;

    // Hardware
    private final DcMotorEx shooterMotor;

    public Shooter(DcMotorEx shooterMotor, GamepadMap map, OpMode opmode) {
        this.shooterMotor = shooterMotor;
        this.map = map;
        this.opmode = opmode;
    }

    public void OperateShooter() {
        shooterMotor.setPower(map.shooterButton);
        if (TELEMETRY_ENABLED) { addTelemetry(); }
    }

    private void addTelemetry() {
        opmode.telemetry.addLine("--- SHOOTER ---");
        opmode.telemetry.addData("Output RPM:", shooterMotor.getVelocity() * 60.0 / 28.0);
        opmode.telemetry.addData("Motor RPM:", shooterMotor.getVelocity() * 60.0 / TPRShooter);
        opmode.telemetry.addData("Shooter Motor Power:", shooterMotor.getPower());
    }
}
