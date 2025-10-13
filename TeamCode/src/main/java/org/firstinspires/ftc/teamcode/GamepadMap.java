package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.ROT_DB;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.STICK_DB;
import static org.firstinspires.ftc.teamcode.util.MathUtil.deadband;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.EdgeTrigger;

@Configurable
public class GamepadMap {
    private final OpMode opmode;

    public double forward, strafe, rotate;
    public boolean angleLockToggle, slowModeToggle, fieldCentricToggle,
            stateEstimatorFallbackToggle, intakeToggle, intakeReverseToggle;
    public double shooterTrigger;
    public boolean resetPinpointButton;
    public double hoodAxis;

    private final EdgeTrigger a = new EdgeTrigger(), b = new EdgeTrigger(), x = new EdgeTrigger(),
            dpad_down_t = new EdgeTrigger(), right_bumper_t = new EdgeTrigger(),
            left_bumper_t = new EdgeTrigger();

    public GamepadMap(OpMode opmode) {
        this.opmode = opmode;
    }

    // TODO: Make final mapping
    public void update() {
        forward = deadband(opmode.gamepad1.left_stick_y, STICK_DB);
        strafe = deadband(opmode.gamepad1.left_stick_x, STICK_DB);
        rotate = deadband(opmode.gamepad1.right_stick_x, ROT_DB);

        angleLockToggle = a.rose(opmode.gamepad1.a);
        slowModeToggle = b.rose(opmode.gamepad1.b);
        stateEstimatorFallbackToggle = x.rose(opmode.gamepad1.x);
        fieldCentricToggle = dpad_down_t.rose(opmode.gamepad1.dpad_down);

        intakeToggle = right_bumper_t.rose(opmode.gamepad1.right_bumper);
        intakeReverseToggle = left_bumper_t.rose(opmode.gamepad1.left_bumper);

        shooterTrigger = opmode.gamepad1.left_trigger;
        resetPinpointButton = opmode.gamepad1.start;

        hoodAxis = opmode.gamepad1.right_trigger - opmode.gamepad1.left_trigger;
    }
}