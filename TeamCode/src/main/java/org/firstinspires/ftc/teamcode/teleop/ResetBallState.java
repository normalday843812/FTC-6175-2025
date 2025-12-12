package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.managers.PersistentBallState;

/**
 * Simple OpMode to reset the persistent ball state.
 * Press Start to reset and clear all ball positions.
 */
@TeleOp(name = "Reset Ball State", group = "Utility")
public class ResetBallState extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== BALL STATE RESET ===");
        telemetry.addLine("");
        telemetry.addData("Current state", PersistentBallState.getStateString());
        telemetry.addData("Ball count", PersistentBallState.getBallCount());
        telemetry.addData("Initialized", PersistentBallState.isInitialized());
        telemetry.addLine("");
        telemetry.addLine("Press START to reset ball state to empty.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            PersistentBallState.reset();

            telemetry.addLine("=== BALL STATE RESET ===");
            telemetry.addLine("");
            telemetry.addLine("STATE CLEARED!");
            telemetry.addLine("");
            telemetry.addData("New state", PersistentBallState.getStateString());
            telemetry.addData("Ball count", PersistentBallState.getBallCount());
            telemetry.update();

            // Wait a moment so user can see the result
            sleep(2000);
        }
    }
}
