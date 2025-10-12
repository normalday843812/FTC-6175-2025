package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.TELEMETRY_ENABLED;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.TPR_MOTOR;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterConstants.TPR_OUTPUT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.GamepadMap;

public class Shooter {
    OpMode opmode;
    GamepadMap map;
    private final DcMotorEx shooterMotor;
    private double tps;
    private double motorRPM;
    private double outputRPM;
    private double targetRpm = 0.0;

    public Shooter(DcMotorEx shooterMotor, GamepadMap map, OpMode opmode) {
        this.shooterMotor = shooterMotor;
        this.map = map;
        this.opmode = opmode;
    }

    public void operateShooter() {
        if (map.shooterButton > 0.0) {
            targetRpm = map.shooterButton * MAX_RPM;
        }
        setRpm(targetRpm);

        tps = shooterMotor.getVelocity();
        motorRPM = tps * 60.0 / TPR_MOTOR;
        outputRPM = tps * 60.0 / TPR_OUTPUT;
        if (TELEMETRY_ENABLED) { addTelemetry(); }
    }

    public void setRpm(double rpm) {
        rpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
        shooterMotor.setVelocity(rpm * TPR_OUTPUT / 60.0);
    }

    private void addTelemetry() {
        double tgtTps = targetRpm * TPR_OUTPUT / 60.0;
        opmode.telemetry.addLine("--- SHOOTER ---");
        opmode.telemetry.addData("Target RPM:", targetRpm);
        opmode.telemetry.addData("Output RPM:", outputRPM);
        opmode.telemetry.addData("Motor RPM:",  motorRPM);
        opmode.telemetry.addData("err tps:", tgtTps - tps);
    }
}
