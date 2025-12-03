package org.firstinspires.ftc.teamcode;

import static com.pedropathing.math.MathFunctions.clamp;
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
            spindexerForward, spindexerBackward, intakeToggle, intakeReverseToggle,
            teleopSortManagerToggle;
    public double shooterUp, shooterDown;
    public double shooterYaw;
    public boolean transferButton, transferButtonHeld, intakeClearJam, resetShooterYaw;
    public boolean transferCrForward, transferCrReverse;
    public double hoodAxis;
    public boolean shooterYawAutoLockToggle;
    public boolean shootingModeToggle, shootingModeToggleHeld;
    public boolean findGreenBall, findPurpleBall;
    private final EdgeTrigger a_gp2 = new EdgeTrigger(), b_gp2 = new EdgeTrigger(), x = new EdgeTrigger(),
            y = new EdgeTrigger(), dpad_down_t = new EdgeTrigger(), right_bumper_t = new EdgeTrigger(),
            dpad_up_t_gp1 = new EdgeTrigger(), dpad_up_t_gp2 = new EdgeTrigger(), dpad_left_t = new EdgeTrigger(), dpad_right_t = new EdgeTrigger(),
            left_bumper_t = new EdgeTrigger(), start_button_t = new EdgeTrigger(),
            back_button_t_gp2 = new EdgeTrigger(),
            left_stick_button_t = new EdgeTrigger(), right_stick_button_t = new EdgeTrigger(),
            a_gp1 = new EdgeTrigger();

    public GamepadMap(OpMode opmode) {
        this.opmode = opmode;
    }

    public void update() {
        // ==================== GAMEPAD 1: DRIVER ====================
        // --- Movement (left stick + right stick X) ---
        forward = deadband(clamp(opmode.gamepad1.left_stick_y, -1.0, 1.0), STICK_DB);
        strafe = deadband(clamp(opmode.gamepad1.left_stick_x, -1.0, 1.0), STICK_DB);
        rotate = deadband(clamp(opmode.gamepad1.right_stick_x, -1.0, 1.0), ROT_DB);

        // --- Drive Mode Toggles (dpad) ---
        fieldCentricToggle = dpad_down_t.rose(opmode.gamepad1.dpad_down);
        slowModeToggle = dpad_up_t_gp1.rose(opmode.gamepad1.dpad_up);
        angleLockToggle = dpad_left_t.rose(opmode.gamepad1.dpad_left);

        // --- Ball Seeking (face buttons - colors match buttons) ---
        findGreenBall = a_gp1.rose(opmode.gamepad1.a);  // A is green
        findPurpleBall = b_gp2.rose(opmode.gamepad1.b); // B is red/close to purple

        // ==================== GAMEPAD 2: OPERATOR ====================
        // --- Intake (bumpers - R=in, L=reverse, intuitive) ---
        intakeToggle = right_bumper_t.rose(opmode.gamepad2.right_bumper);
        intakeReverseToggle = left_bumper_t.rose(opmode.gamepad2.left_bumper);
        intakeClearJam = start_button_t.rose(opmode.gamepad2.start);

        // --- Shooter Power (triggers - variable control) ---
        shooterUp = opmode.gamepad2.right_trigger;
        shooterDown = opmode.gamepad2.left_trigger;

        // --- Shooter Aiming (right stick - dedicated axis) ---
        shooterYaw = deadband(opmode.gamepad2.right_stick_x, ROT_DB);
        shooterYawAutoLockToggle = right_stick_button_t.rose(opmode.gamepad2.right_stick_button);
        resetShooterYaw = left_stick_button_t.rose(opmode.gamepad2.left_stick_button);

        // --- Transfer/Spindexer (dpad - directional logic) ---
        transferButton = dpad_up_t_gp2.rose(opmode.gamepad2.dpad_up)
                || dpad_up_t_gp1.rose(opmode.gamepad1.dpad_up);
        transferButtonHeld = opmode.gamepad2.dpad_up;
        transferCrForward = dpad_right_t.rose(opmode.gamepad2.dpad_right);
        transferCrReverse = dpad_left_t.rose(opmode.gamepad2.dpad_left);
        spindexerForward = x.rose(opmode.gamepad2.x);
        spindexerBackward = y.rose(opmode.gamepad2.y);

        // --- Shooting Mode (face button A - primary action) ---
        shootingModeToggle = a_gp2.rose(opmode.gamepad2.a);
        shootingModeToggleHeld = opmode.gamepad2.a;

        // --- Manager Toggles (back/start - rarely used) ---
        teleopSortManagerToggle = back_button_t_gp2.rose(opmode.gamepad2.back);
    }
}
