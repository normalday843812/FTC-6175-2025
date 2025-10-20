package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GamepadMap;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.util.SubsystemMode;
import org.firstinspires.ftc.teamcode.util.TelemetryHelper;

public class Intake {
    public enum AutoMode {OFF, FORWARD, REVERSE}

    private final DcMotor motor;
    private final GamepadMap map;
    private boolean running = false;
    private boolean reverse = false;
    private final TelemetryHelper tele;

    private SubsystemMode mode = SubsystemMode.MANUAL;

    private AutoMode autoMode = AutoMode.OFF;

    public Intake(DcMotor motor, GamepadMap map, OpMode opmode) {
        this.motor = motor;
        this.map = map;
        this.tele = new TelemetryHelper(opmode, IntakeConfig.TELEMETRY_ENABLED);
    }

    public void startTeleop() {
        mode = SubsystemMode.MANUAL;
    }

    public void startAuto() {
        mode = SubsystemMode.AUTO;
        autoMode = AutoMode.OFF;
    }

    public void setAutoMode(AutoMode m) {
        this.autoMode = m;
    }

    public void operate() {
        if (mode == SubsystemMode.MANUAL) {
            operateManual();
        } else {
            operateAuto();
        }
        addTelemetry();
    }

    private void operateManual() {
        if (map != null) {
            if (map.intakeToggle) running = !running;
            if (map.intakeReverseToggle) reverse = !reverse;
        }
        double power = running ? (reverse ? IntakeConfig.REVERSE_PWR : IntakeConfig.FORWARD_PWR) : 0.0;
        motor.setPower(power);
    }

    private void operateAuto() {
        switch (autoMode) {
            case OFF:
                motor.setPower(0.0);
                break;
            case FORWARD:
                motor.setPower(IntakeConfig.FORWARD_PWR);
                break;
            case REVERSE:
                motor.setPower(IntakeConfig.REVERSE_PWR);
                break;
        }
    }

    private void addTelemetry() {
        tele.addLine("--- INTAKE ---")
                .addData("Mode", mode::name)
                .addData("AutoMode", autoMode::name)
                .addData("Power:", "%.2f", motor.getPower());
    }
}
