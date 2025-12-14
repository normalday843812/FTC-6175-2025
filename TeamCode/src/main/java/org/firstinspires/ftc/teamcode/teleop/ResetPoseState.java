package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.AutoConfig;
import org.firstinspires.ftc.teamcode.config.DecodeGameConfig;
import org.firstinspires.ftc.teamcode.managers.PersistentPoseState;

/**
 * Simple OpMode to reset the persistent pose state to the configured starting pose.
 *
 * <p>Uses {@link AutoConfig#isRed} and {@link AutoConfig#isAudienceSide} to choose the start pose.</p>
 */
@TeleOp(name = "Reset Pose State", group = "Utility")
public class ResetPoseState extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("=== POSE STATE RESET ===");
        telemetry.addLine("");
        telemetry.addData("Initialized", "%b", PersistentPoseState.isInitialized());
        telemetry.addData("Current pose", "%s", PersistentPoseState.getPoseString());
        telemetry.addLine("");
        telemetry.addData("Config isRed", "%b", AutoConfig.isRed);
        telemetry.addData("Config audienceSide", "%b", AutoConfig.isAudienceSide);
        telemetry.addLine("");
        telemetry.addLine("Press START to reset pose state to DecodeGameConfig.startPose().");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            Pose startPose = DecodeGameConfig.startPose(AutoConfig.isRed, AutoConfig.isAudienceSide);
            PersistentPoseState.saveFromPose(startPose);

            telemetry.addLine("=== POSE STATE RESET ===");
            telemetry.addLine("");
            telemetry.addLine("POSE SAVED!");
            telemetry.addLine("");
            telemetry.addData("New pose", "%s", PersistentPoseState.getPoseString());
            telemetry.update();

            sleep(2000);
        }
    }
}

