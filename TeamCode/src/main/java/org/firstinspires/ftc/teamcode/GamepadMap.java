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
    public boolean angleLockToggle, slowModeToggle, fieldCentricToggle, shooterManagerToggle,
            spindexerForward, spindexerBackward, intakeToggle, intakeReverseToggle;
    public double shooterUp, shooterDown;
    public double shooterYaw;
    public boolean transferButton, intakeClearJam;
    public double hoodAxis;

    private final EdgeTrigger a = new EdgeTrigger(), b = new EdgeTrigger(), x = new EdgeTrigger(),
            y = new EdgeTrigger(), dpad_down_t = new EdgeTrigger(), right_bumper_t = new EdgeTrigger(),
            dpad_up_t = new EdgeTrigger(), dpad_left_t = new EdgeTrigger(), dpad_right_t = new EdgeTrigger(),
            left_bumper_t = new EdgeTrigger(), start_button_t = new EdgeTrigger(), back_button_t = new EdgeTrigger();

    public GamepadMap(OpMode opmode) {
        this.opmode = opmode;
    }

    public void update() {
        forward = deadband(opmode.gamepad1.left_stick_y, STICK_DB);
        strafe = deadband(opmode.gamepad1.left_stick_x, STICK_DB);
        rotate = deadband(opmode.gamepad1.right_stick_x, ROT_DB);

        spindexerForward = x.rose(opmode.gamepad1.x);
        spindexerBackward = y.rose(opmode.gamepad1.y);

        angleLockToggle = a.rose(opmode.gamepad1.a);
        slowModeToggle = b.rose(opmode.gamepad1.b);
        fieldCentricToggle = dpad_down_t.rose(opmode.gamepad1.dpad_down);

        shooterManagerToggle = back_button_t.rose(opmode.gamepad1.back);

        intakeToggle = right_bumper_t.rose(opmode.gamepad1.right_bumper);
        intakeReverseToggle = left_bumper_t.rose(opmode.gamepad1.left_bumper);
        intakeClearJam = start_button_t.rose(opmode.gamepad1.start);

        shooterUp = opmode.gamepad1.right_trigger;
        shooterDown = opmode.gamepad1.left_trigger;

        shooterYaw = deadband(opmode.gamepad1.right_stick_y, ROT_DB);

        transferButton = dpad_up_t.rose(opmode.gamepad1.dpad_up);

        hoodAxis = opmode.gamepad1.right_trigger - opmode.gamepad1.left_trigger;
    }
}
