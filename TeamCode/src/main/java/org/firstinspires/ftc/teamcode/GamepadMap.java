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
    public boolean angleLockToggle, slowModeToggle, fieldCentricToggle, shooterManagerToggle,
            spindexerForward, spindexerBackward, intakeToggle, intakeReverseToggle,
            teleopSortManagerToggle;
    public double shooterUp, shooterDown;
    public double shooterYaw;
    public boolean transferButton, transferButtonHeld, intakeClearJam, resetShooterYaw;
    public boolean transferCrForward, transferCrReverse;
    public double hoodAxis;
    public boolean shooterYawAutoLockToggle;
    public boolean shootingModeToggle;
    public boolean findGreenBall, findPurpleBall;
    private final EdgeTrigger a_gp2 = new EdgeTrigger(), b_gp2 = new EdgeTrigger(), x = new EdgeTrigger(),
            y = new EdgeTrigger(), dpad_down_t = new EdgeTrigger(), right_bumper_t = new EdgeTrigger(),
            dpad_up_t = new EdgeTrigger(), dpad_left_t = new EdgeTrigger(), dpad_right_t = new EdgeTrigger(),
            left_bumper_t = new EdgeTrigger(), start_button_t = new EdgeTrigger(), back_button_t_gp1 = new EdgeTrigger(),
            back_button_t_gp2 = new EdgeTrigger(),
            left_stick_button_t = new EdgeTrigger(), right_stick_button_t = new EdgeTrigger(),
            a_gp1 = new EdgeTrigger();
    public GamepadMap(OpMode opmode) {
        this.opmode = opmode;
    }

    public void update() {
        forward = deadband(
                clamp(opmode.gamepad2.left_stick_y + opmode.gamepad1.left_stick_y,
                        -1.0, 1.0)
                , STICK_DB);
        strafe = deadband(clamp(opmode.gamepad2.left_stick_x + opmode.gamepad1.left_stick_x,
                -1.0, 1.0), STICK_DB);
        rotate = deadband(clamp(opmode.gamepad2.right_stick_x + opmode.gamepad1.right_stick_x,
                -1.0, 1.0), ROT_DB);

        spindexerForward = x.rose(opmode.gamepad1.x);
        spindexerBackward = y.rose(opmode.gamepad1.y);

        shootingModeToggle = a_gp1.rose(opmode.gamepad1.a);

        transferCrForward = dpad_right_t.rose(opmode.gamepad1.dpad_right);
        transferCrReverse = dpad_left_t.rose(opmode.gamepad1.dpad_left);

        findGreenBall = a_gp2.rose(opmode.gamepad2.a);
        findPurpleBall = b_gp2.rose(opmode.gamepad2.b);

        fieldCentricToggle = dpad_down_t.rose(opmode.gamepad2.dpad_down);

        shooterManagerToggle = back_button_t_gp1.rose(opmode.gamepad1.back);
        teleopSortManagerToggle = back_button_t_gp2.rose(opmode.gamepad2.back);

        intakeToggle = right_bumper_t.rose(opmode.gamepad1.right_bumper);
        intakeReverseToggle = left_bumper_t.rose(opmode.gamepad1.left_bumper);
        intakeClearJam = start_button_t.rose(opmode.gamepad1.start);
        resetShooterYaw = left_stick_button_t.rose(opmode.gamepad1.left_stick_button);

        shooterYawAutoLockToggle = right_stick_button_t.rose(opmode.gamepad1.right_stick_button);
        shooterUp = opmode.gamepad1.right_trigger;
        shooterDown = opmode.gamepad1.left_trigger;

        shooterYaw = deadband(opmode.gamepad1.right_stick_y, ROT_DB);

        transferButton = dpad_up_t.rose(opmode.gamepad1.dpad_up);
        transferButtonHeld = opmode.gamepad1.dpad_up;

        hoodAxis = opmode.gamepad1.right_trigger - opmode.gamepad1.left_trigger;
    }
}
