package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Gamepad Test", group = "Testing")
public class GamepadTest extends LinearOpMode {
    private static final double EPS = 0.05;

    private void addIfTrue(String label, boolean value) {
        if (value) telemetry.addData(label, true);
    }

    private void addIfNonZero(String label, float value) {
        if (Math.abs(value) > EPS) telemetry.addData(label, "%.3f", value);
    }

    private void logGamepad(String name, Gamepad gp) {
        // --- Buttons (booleans) ---
        addIfTrue(name + " a", gp.a);
        addIfTrue(name + " b", gp.b);
        addIfTrue(name + " x", gp.x);
        addIfTrue(name + " y", gp.y);

        addIfTrue(name + " dpad_up", gp.dpad_up);
        addIfTrue(name + " dpad_down", gp.dpad_down);
        addIfTrue(name + " dpad_left", gp.dpad_left);
        addIfTrue(name + " dpad_right", gp.dpad_right);

        addIfTrue(name + " left_bumper", gp.left_bumper);
        addIfTrue(name + " right_bumper", gp.right_bumper);

        addIfTrue(name + " left_stick_button", gp.left_stick_button);
        addIfTrue(name + " right_stick_button", gp.right_stick_button);

        addIfTrue(name + " back", gp.back);
        addIfTrue(name + " start", gp.start);
        addIfTrue(name + " guide", gp.guide);

        // --- Analogs (floats) ---
        addIfNonZero(name + " left_stick_x", gp.left_stick_x);
        addIfNonZero(name + " left_stick_y", gp.left_stick_y);
        addIfNonZero(name + " right_stick_x", gp.right_stick_x);
        addIfNonZero(name + " right_stick_y", gp.right_stick_y);

        addIfNonZero(name + " left_trigger", gp.left_trigger);
        addIfNonZero(name + " right_trigger", gp.right_trigger);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            // Report only active inputs for BOTH controllers
            logGamepad("gamepad1", gamepad1);
            logGamepad("gamepad2", gamepad2);

            telemetry.update();
            idle(); // be polite to the scheduler
        }
    }
}
