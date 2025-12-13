package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.clamp;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.ROT_DB;
import static org.firstinspires.ftc.teamcode.config.DriveConfig.STICK_DB;
import static org.firstinspires.ftc.teamcode.util.MathUtil.deadband;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.EdgeTrigger;

/**
 * Single-gamepad control scheme.
 *
 * Layout:
 * - Left stick: Drive (forward/strafe)
 * - Right stick X: Rotate
 * - Right stick Y: Shooter yaw manual adjust
 * - Triggers: Shooter power (RT=up, LT=down)
 * - Bumpers: Shooter yaw trim (TeleopManager ON), intake hold-controls (TeleopManager OFF)
 * - A: Hold lever up (manual mode)
 * - B: Spindexer backward
 * - X: Spindexer forward
     * - Y: Slow mode toggle
     * - Dpad up: Transfer shoot (flicks ball out)
 * - Dpad down: Field centric toggle
 * - Dpad left: Transfer CR reverse
 * - Dpad right: Transfer CR forward
 * - Left stick button: Reset shooter yaw
 * - Right stick button: Shooter yaw auto lock toggle
 * - Back: Teleop manager toggle
 * - Start: (unused - jam clearing is automatic)
     */
@Configurable
public class GamepadMap {
    private final OpMode opmode;

    // Drive
    public double forward, strafe, rotate;
    public boolean slowModeToggle, fieldCentricToggle;

    // Intake
    public boolean intakeToggle, intakeReverseToggle;

    // Shooter
    public double shooterUp, shooterDown;
    public double shooterYaw;
    public boolean shooterYawAutoLockToggle, resetShooterYaw;
    public boolean shooterYawBiasInc, shooterYawBiasDec;

    // Transfer/Spindexer
    public boolean transferButton, transferButtonHeld;  // Flicks ball out (shoot); held is unused (kept for compat)
    public boolean transferCrForward, transferCrReverse;
    public boolean spindexerForward, spindexerBackward;

    // Modes
    public boolean leverHeld;  // Hold transfer lever up (manual mode)
    public boolean teleopSortManagerToggle;
    public boolean clearAll; // Resets ball state/model and stops mechanisms

    // Hood (if using right stick Y for hood instead of shooter yaw)
    public double hoodAxis;

    // Edge triggers
    private final EdgeTrigger b_t = new EdgeTrigger();
    private final EdgeTrigger x_t = new EdgeTrigger();
    private final EdgeTrigger y_t = new EdgeTrigger();
    private final EdgeTrigger dpad_up_t = new EdgeTrigger();
    private final EdgeTrigger dpad_down_t = new EdgeTrigger();
    private final EdgeTrigger dpad_left_t = new EdgeTrigger();
    private final EdgeTrigger dpad_right_t = new EdgeTrigger();
    private final EdgeTrigger rb_t = new EdgeTrigger();
    private final EdgeTrigger lb_t = new EdgeTrigger();
    private final EdgeTrigger back_t = new EdgeTrigger();
    private final EdgeTrigger start_t = new EdgeTrigger();
    private final EdgeTrigger ls_button_t = new EdgeTrigger();
    private final EdgeTrigger rs_button_t = new EdgeTrigger();

    public GamepadMap(OpMode opmode) {
        this.opmode = opmode;
    }

    public void update() {
        // === DRIVE (Left stick + Right stick X) ===
        forward = deadband(clamp(opmode.gamepad1.left_stick_y, -1.0, 1.0), STICK_DB);
        strafe = deadband(clamp(opmode.gamepad1.left_stick_x, -1.0, 1.0), STICK_DB);
        rotate = deadband(clamp(opmode.gamepad1.right_stick_x, -1.0, 1.0), ROT_DB);

        // === DRIVE MODE TOGGLES ===
        slowModeToggle = y_t.rose(opmode.gamepad1.y);
        fieldCentricToggle = dpad_down_t.rose(opmode.gamepad1.dpad_down);

        // === SHOOTER YAW BIAS (Bumpers) ===
        shooterYawBiasInc = rb_t.rose(opmode.gamepad2.right_bumper);
        shooterYawBiasDec = lb_t.rose(opmode.gamepad2.left_bumper);

        // Manual intake (only used when TeleopManager is OFF).
        // LEFT bumper = intake (REVERSE), RIGHT bumper = block/eject (FORWARD).
        intakeReverseToggle = opmode.gamepad1.left_bumper;
        intakeToggle = opmode.gamepad1.right_bumper;

        // === SHOOTER POWER (Triggers) ===
        shooterUp = opmode.gamepad1.right_trigger;
        shooterDown = opmode.gamepad1.left_trigger;

        // === SHOOTER YAW (Right stick Y + buttons) ===
        shooterYaw = deadband(opmode.gamepad1.right_stick_y, ROT_DB);
        shooterYawAutoLockToggle = rs_button_t.rose(opmode.gamepad1.right_stick_button);
        resetShooterYaw = ls_button_t.rose(opmode.gamepad1.left_stick_button);

        // === TRANSFER SHOOT (Dpad up) ===
        transferButton = dpad_up_t.rose(opmode.gamepad1.dpad_up);
        transferButtonHeld = false;

        // === TRANSFER CR (Dpad left/right) ===
        transferCrForward = dpad_right_t.rose(opmode.gamepad1.dpad_right);
        transferCrReverse = dpad_left_t.rose(opmode.gamepad1.dpad_left);

        // === SPINDEXER (X/B) ===
        spindexerForward = x_t.rose(opmode.gamepad1.x);
        spindexerBackward = b_t.rose(opmode.gamepad1.b);

        // === LEVER (A) ===
        leverHeld = opmode.gamepad1.a;

        // === MANAGER TOGGLE (Back) ===
        teleopSortManagerToggle = back_t.rose(opmode.gamepad1.back);

        // === CLEAR ALL (Start) ===
        clearAll = start_t.rose(opmode.gamepad1.start);

        // === HOOD (unused axis, available if needed) ===
        hoodAxis = 0; // Not mapped currently
    }
}
