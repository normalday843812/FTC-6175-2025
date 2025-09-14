package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Configurable
public class GamepadMap {
    private final OpMode opmode;

    public double forward;
    public double strafe;
    public double rotate;
    public boolean angleLockToggle;
    public boolean slowModeToggle;
    public boolean fieldCentricToggle;
    public boolean intakeToggle;
    public boolean resetPinpointButton;

    public GamepadMap(OpMode opmode) {
        this.opmode = opmode;
    }

    public void update() {
        forward = -opmode.gamepad1.left_stick_y;
        strafe = opmode.gamepad1.left_stick_x;
        rotate = opmode.gamepad1.right_stick_x;
        angleLockToggle = opmode.gamepad1.a;
        slowModeToggle = opmode.gamepad1.b;
        fieldCentricToggle = opmode.gamepad1.x;
        intakeToggle = opmode.gamepad1.right_trigger == 1.0;
        resetPinpointButton = opmode.gamepad1.start;
    }
}